
from typing import Dict, List
import numpy as np
import torch

from pydispatch import Dispatcher
from beartype import beartype


from multi_camera_driver.helpers.image_handler import CameraImage
from multi_camera_driver.helpers.common import CameraSettings, EncoderError, bayer_pattern, encoding_bits
from multi_camera_driver.helpers.image_processor.outputs import ImageOutputs
from multi_camera_driver.helpers.image_processor.util import TiQueue, taichi_pattern
from multi_camera_driver.helpers.image_settings import ImageSettings
from multi_camera_driver.helpers.work_queue import WorkQueue

from taichi_image import camera_isp, interpolate
import taichi as ti
import gc


def common_value(name, values):
  assert len(set(values)) == 1, f"All cameras must have the same {name}"
  return values[0]

class FrameProcessor(Dispatcher):
  _events_ = ["on_frame"]

  @beartype
  def __init__(self, cameras:Dict[str, CameraSettings], settings:ImageSettings):
    self.settings = settings
    self.cameras = cameras

    self.processor = TiQueue.run_sync(self.init_processor, cameras)

    self.queue = WorkQueue("FrameProcessor", run=self.process_worker, num_workers=4, max_size=4)
    self.queue.start()

  def update_camera(self, k, camera):
    self.cameras[k] = camera

  def update_settings(self, settings:ImageSettings):
    self.settings = settings
    
    self.isp.set(moving_alpha=self.settings.moving_average, 
                 resize_width=int(self.settings.resize_width),
                 transform=interpolate.ImageTransform(self.settings.transform))


  @beartype
  def init_processor(self, cameras:Dict[str, CameraSettings]):
    enc = common_value("encoding", [camera.encoding for camera in cameras.values()])    
    
    self.pattern = bayer_pattern(enc)
    self.bits = encoding_bits(enc)

    if encoding_bits(enc) not in [12, 16]:
      raise ValueError(f"Unsupported bits {encoding_bits(enc)} in {enc}")

    self.isp = camera_isp.Camera16(taichi_pattern[self.pattern], 
                         resize_width=int(self.settings.resize_width), 
                        moving_alpha=self.settings.moving_average,
                        transform=interpolate.ImageTransform(self.settings.transform),
                        device=torch.device(self.settings.device))


  @beartype
  def process(self, images:Dict[str, CameraImage]):
    return self.queue.enqueue(images)


  def upload_image(self, image:torch.Tensor):
    assert image.dtype == torch.uint8
    
    return image.to(device=self.settings.device, non_blocking=True)
  

  @beartype
  def process_worker(self, camera_images:Dict[str, CameraImage]):
    images = [self.upload_image(image.image_data) 
              for image in camera_images.values()]

    images, previews = TiQueue.run_sync(self.process_images, images)

    outputs = [ImageOutputs(
      raw = camera_images[k], 
      rgb = image, 
      preview = preview, 
      settings = self.settings,
      calibration=self.cameras[k].calibration)
                    for k, image, preview in zip(camera_images.keys(), images, previews)]

    self.emit("on_frame", outputs)

  # @beartype
  def process_images(self, images:List[torch.Tensor]):
    load_data = self.isp.load_packed12 if self.bits == 12 else self.isp.load_packed16
    images =  [load_data(image) for image in images]

    if self.settings.tone_mapping == "linear":
      outputs = self.isp.tonemap_linear(images, gamma=self.settings.tone_gamma)
    elif self.settings.tone_mapping == "reinhard":
      outputs = self.isp.tonemap_reinhard(
        images, gamma=self.settings.tone_gamma, 
        intensity = self.settings.tone_intensity,
        light_adapt = self.settings.light_adapt,
        color_adapt = self.settings.color_adapt)
    else:
      raise ValueError(f"Unknown tone mapper {self.settings.tone_mapper}")

    previews = [interpolate.resize_width(output, self.settings.preview_size) for output in outputs]
    return outputs, previews


  def stop(self):
    self.queue.stop()
