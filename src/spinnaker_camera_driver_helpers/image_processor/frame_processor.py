
from typing import Dict, List
import numpy as np
import torch

from pydispatch import Dispatcher
from beartype import beartype

from nvjpeg_torch import Jpeg

from spinnaker_camera_driver_helpers.image_handler import CameraImage
from spinnaker_camera_driver_helpers.common import CameraSettings, EncoderError, bayer_pattern
from spinnaker_camera_driver_helpers.image_processor.outputs import ImageOutputs
from spinnaker_camera_driver_helpers.image_processor.util import TiQueue, resize_width, taichi_pattern
from spinnaker_camera_driver_helpers.image_settings import ImageSettings
from spinnaker_camera_driver_helpers.work_queue import WorkQueue

from taichi_image import camera_isp, interpolate
import taichi as ti


def common_value(name, values):
  assert len(set(values)) == 1, f"All cameras must have the same {name}"
  return values[0]

class FrameProcessor(Dispatcher):
  _events_ = ["on_frame"]

  @beartype
  def __init__(self, cameras:Dict[str, CameraSettings], settings:ImageSettings):
    self.jpeg = Jpeg()
    self.settings = settings
    self.cameras = cameras

    self.processor = TiQueue.run_sync(self.init_processor, cameras)

    self.queue = WorkQueue("FrameProcessor", run=self.process_worker, max_size=1)
    self.queue.start()

  def update_camera(self, k, camera):
    self.processors[k].update_camera(camera)

  def update_settings(self, settings:ImageSettings):

    for processor in self.processors:
      processor.update_settings(settings)


  @beartype
  def init_processor(self, cameras:Dict[str, CameraSettings]):
    camera_encoding = common_value("encoding", [camera.encoding for camera in cameras.values()])    
  
    pattern = bayer_pattern(camera_encoding)
    CameraISP = camera_isp.camera_isp(ti.f16)

    self.isp = CameraISP(taichi_pattern[pattern], 
                         resize_width=self.settings.resize_width, 
                        moving_alpha=self.settings.moving_average)


  @beartype
  def process(self, images:Dict[str, CameraImage]):
    return self.queue.enqueue(images)

  def encode(self, image:torch.Tensor):
    try:
      return self.jpeg.encode(image,
                              quality=self.settings.jpeg_quality,
                              input_format=Jpeg.RGBI).numpy().tobytes()

    except Jpeg.Exception as e:
      raise EncoderError(str(e))


  @beartype
  def process_worker(self, camera_images:Dict[str, CameraImage]):
    images = [torch.from_numpy(image.image_data).to(device=self.settings.device) 
              for image in camera_images.values()]
  
    # images = [image.image_data for image in camera_images.values()]
    images, previews = TiQueue.run_sync(self.process_images, images)

    outputs = [ImageOutputs(camera_images[k], image, preview, encode=self.encode, calibration=self.cameras[k].calibration)
                    for k, image, preview in zip(camera_images.keys(), images, previews)]


    self.emit("on_frame", outputs)
  

  # @beartype
  def process_images(self, images:List[torch.Tensor]):

    images = [self.isp.load_packed12(image.view(-1), camera.image_size) for image, camera in zip(images, self.cameras.values())]
    outputs = self.isp.tonemap_linear(images, gamma=self.settings.tone_gamma)

    _, preview_size = resize_width(self.isp.output_size, self.settings.preview_size)
    previews = [interpolate.resize_bilinear(output, preview_size) for output in outputs]

    return outputs, previews


  def stop(self):
    self.queue.stop()
