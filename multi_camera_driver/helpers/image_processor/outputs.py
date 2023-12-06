
from dataclasses import dataclass
import threading
from typing import Callable, Optional
from beartype import beartype
from cached_property import cached_property

from nvjpeg_torch import Jpeg
from taichi_image import color

from multi_camera_driver.helpers.common import CameraImage, EncoderError
from multi_camera_driver.helpers.geometry_conversions import camera_info_msg
from camera_geometry import Camera
from sensor_msgs.msg import CameraInfo
from multi_camera_driver.helpers.image_settings import ImageSettings
#from exif import Image


import torch

local_jpeg = threading.local()

def jpeg():

  if not hasattr(local_jpeg, "encoder"):
    local_jpeg.encoder = Jpeg()
  return local_jpeg.encoder





@beartype
@dataclass
class ImageOutputs(object):
    
  raw:CameraImage
  rgb:torch.Tensor
  preview:torch.Tensor
  settings : ImageSettings

  calibration:Optional[Camera] = None

  def encode(self, image:torch.Tensor):
    try:
      encoder = jpeg()

      
      if hasattr(encoder, 'encode'):
        return encoder.encode(image,
                              quality=self.settings.jpeg_quality,
                              input_format=Jpeg.RGBI).numpy().tobytes()
      elif hasattr(encoder, 'encode_yuv_420'):
        from multi_camera_driver.helpers.image_processor import  TiQueue


        yuv = TiQueue.run_sync(color.rgb_yuv420_image, image)
        bytes = encoder.encode_yuv_420(yuv.to('cpu', non_blocking=True),
                                  quality=self.settings.jpeg_quality).numpy().tobytes()
        return bytes
      else:
        raise EncoderError(f"Encoder does not support encode or encode_yuv_420: {encoder}")
      

    except Jpeg.Exception as e:
      raise EncoderError(str(e))

  @cached_property
  def compressed(self) -> bytes:
    return self.encode(self.rgb)

  @cached_property
  def compressed_preview(self) -> bytes:
    return self.encode(self.preview)
  
  @property
  def camera_name(self) -> str:
    return self.raw.camera_name

  @property
  def camera_info(self) -> CameraInfo:
    height, width, _ = self.rgb.shape

    if self.calibration is not None:  
      if self.calibration.width != width or self.calibration.height != height:
        calibration = self.calibration.resize_image( (width, height) )
        
      return camera_info_msg(calibration)

    else:
      return CameraInfo(width = width, height = height)



