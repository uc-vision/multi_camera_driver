

import math
from typing import Dict, List
from beartype import beartype
from cached_property import cached_property
import numpy as np

from nvjpeg_torch import Jpeg, JpegException
import torch
from spinnaker_camera_driver_helpers.image_handler import CameraImage
from spinnaker_camera_driver_helpers.common import CameraSettings, EncoderError, ImageEncoding, bayer_pattern, encoding_bits
from spinnaker_camera_driver_helpers.image_processor.util import TiQueue, resize_longest


import taichi as ti
from taichi_image import bayer, interpolate, tonemap, packed, types

from spinnaker_camera_driver_helpers.image_settings import ImageSettings, PublisherSettings
from .util import load_16f_kernel, taichi_pattern

from py_structs.torch import shape_info



class ImageOutputs(object):

  def __init__(self, image, jpeg, settings:PublisherSettings):
    self.jpeg = jpeg
    self.image = image
    
    self.jpeg_quality = settings.image.jpeg_quality

    

  def encode(self, image):
    try:
      return self.jpeg.encode(image.squeeze(0),
                              quality=self.jpeg_quality,
                              input_format=Jpeg.RGBI).numpy().tobytes()

    except JpegException as e:
      raise EncoderError(str(e))

  @cached_property
  def compressed(self):
    return self.encode(self.image)

  @cached_property
  def preview(self):
    preview_rgb = TiQueue.run(interpolate.resize_bilinear(self.image,
                                              size=self.preview_size, dtype=ti.uint8)).result()
    return self.encode(preview_rgb)


  # def publisher(self):
  #       bridge = CvBridge()

  #   topics = {
  #       "image_raw"        : (Image, lambda data: bridge.cv2_to_imgmsg(data.raw, encoding=settings.camera.encoding.value)),
  #       "compressed"       : (CompressedImage, lambda data: CompressedImage(data = data.compressed, format = "jpeg")), 
  #       "preview/compressed" :  (CompressedImage, lambda data: CompressedImage(data = data.preview, format = "jpeg")),
  #       "camera_info" : (sensor_msgs.msg.CameraInfo, lambda data: data.camera_info)
  #   }

  #   self.publisher = LazyPublisher(topics, self.register, name=self.camera_name)



def encoding(name, camera:CameraSettings):
  enc = camera.encoding
  
  pattern = bayer_pattern(enc)
  bits = encoding_bits(enc)

  if encoding_bits(enc) not in [12, 16]:
    raise ValueError(f"Camera {name}: unsupported bits {encoding_bits(enc)} in {enc}")
  
  return pattern, bits


def resize_width(image_size, resize_width):
  if resize_width > 0:
    scale = resize_width / image_size[0]
    return scale, (resize_width, math.ceil(image_size[1] * scale))
  else:
    return 1.0, image_size


@ti.data_oriented
class CameraProcessor(object):
  def __init__(self, name:str, settings:ImageSettings, camera:CameraSettings, dtype = ti.f16, device = "cuda:0"):
    self.settings = settings
    self.camera = camera
    self.dtype = dtype
    self.device = device
    w, h = camera.image_size

    self.name = name
    self.pattern, self.bits = encoding(name, camera)
    self.min_max = np.array([0, 1])

    self.bayer16 = torch.zeros((h, w), dtype=types.ti_to_torch[self.dtype], device=self.device)
    self.rgb = torch.zeros((h, w, 3), dtype=types.ti_to_torch[self.dtype], device=self.device)

    if self.settings.resize_width > 0:
      self.scale, self.output_size = resize_width(camera.image_size, self.settings.resize_width)
      self.resized = torch.zeros((self.output_size[1], self.output_size[0], 3), dtype=types.ti_to_torch[self.dtype], device=self.device)

    else:
      self.resized = self.rgb
    

    self.bilinear_kernel = interpolate.bilinear_kernel(self.dtype)
    self.linear = ti.field(dtype=self.dtype, shape=self.resized.shape)
    self.bayer_to_rgb = bayer.bayer_to_rgb_kernel(taichi_pattern[self.pattern], self.dtype)

    self.load_kernel = (load_16f_kernel if self.bits == 16  
                         else packed.decode12_kernel(self.dtype, scaled=True))
    
    self.min_max_kernel = tonemap.min_max_kernel(dtype=self.dtype)

  # This is needed because torch doesn't have unsigned integer types
  def load_image(self, image_raw):
    self.load_kernel(image_raw, self.bayer16.view(-1))
    self.bayer_to_rgb(self.bayer16, self.rgb)
    
    if self.scale != 1:
      self.bilinear_kernel(self.rgb, self.resized, scale=ti.math.vec2(self.scale, self.scale))
    self.min_max = np.array(self.min_max_kernel(self.resized))
    

  def outputs(self, image):
    self.preview_scale, self.preview_size = resize_width(self.resized, self.settings.preview_size)
    
    self.preview = torch.empty((self.preview_size[1], self.preview_size[0], 3), dtype=torch.uint8, device=self.device)
    self.output = torch.empty((self.output_size[1], self.output_size[0], 3), dtype=torch.uint8, device=self.device)


def ema(old, new, alpha):
  return new * alpha + old * (1 - alpha)



class ImageProcessor(object):

  @beartype
  def __init__(self, cameras:Dict[str, CameraSettings], settings:ImageSettings):
    self.jpeg = Jpeg()
    self.settings = settings
    self.cameras = cameras

    self.processors = TiQueue.run_sync(self.init_processors, cameras)
    self.min_max = np.array([0, 1])

  @beartype
  def init_processors(self, cameras:Dict[str, CameraSettings]):
    return [CameraProcessor(name, self.settings, camera) for name, camera in cameras.items()]

  @beartype
  def __call__(self, images:Dict[str, CameraImage]) -> List[ImageOutputs]:
    images = [torch.from_numpy(image.image_data).to(device=self.settings.device) 
              for image in images.values()]

    return TiQueue.run_sync(self.process_images, images)

  def process_images(self, images:Dict[str, CameraImage]) -> List[ImageOutputs]:

    for processor, image in zip(self.processors, images):
      processor.load_image(image)

    min_maxs = np.array([processor.min_max for processor in self.processors])
    min_maxs = np.array([min_maxs[0].min(), min_maxs[1].max()])

    self.min_max = ema(self.min_max, min_maxs, self.settings.ema_alpha )


    return []

