
from typing import Dict, List
import numpy as np
import torch

from pydispatch import Dispatcher
from beartype import beartype

from nvjpeg_torch import Jpeg

from spinnaker_camera_driver_helpers.image_handler import CameraImage
from spinnaker_camera_driver_helpers.common import CameraSettings
from spinnaker_camera_driver_helpers.image_processor.outputs import ImageOutputs
from spinnaker_camera_driver_helpers.image_processor.util import TiQueue, ema
from spinnaker_camera_driver_helpers.image_settings import ImageSettings
from .camera_processor import CameraProcessor


class FrameProcessor(Dispatcher):
  _events_ = ["on_frame"]

  @beartype
  def __init__(self, cameras:Dict[str, CameraSettings], settings:ImageSettings):
    self.jpeg = Jpeg()
    self.settings = settings
    self.cameras = cameras

    self.processors = TiQueue.run_sync(self.init_processors, cameras)
    self.min_max = np.array([0, 1])
    self.intensity = 1.0

  def update_camera(self, k, camera):
    self.processors[k].update_camera(camera)

  def update_settings(self, settings:ImageSettings):

    for k in self.processors.keys():
      self.processors[k].update_settings(settings)


  @beartype
  def init_processors(self, cameras:Dict[str, CameraSettings]):
    return [CameraProcessor(name, self.settings, camera, device=self.settings.device) 
            for name, camera in cameras.items()]

  @beartype
  def process(self, images:Dict[str, CameraImage]) -> List[ImageOutputs]:
    images = [torch.from_numpy(image.image_data).to(device=self.settings.device) 
              for image in images.values()]

    return TiQueue.sync(self.upload_images, images)
  

  def upload_images(self, images:Dict[str, CameraImage]) -> List[ImageOutputs]:

    for processor, image in zip(self.processors, images):
      processor.load_image(image)

    min_maxs = np.array([processor.min_max for processor in self.processors])
    min_maxs = np.array([min_maxs[0].min(), min_maxs[1].max()])

    self.min_max = ema(self.min_max, min_maxs, self.settings.ema_alpha)

    return [ImageOutputs(processor.name, raw, *processor.outputs(self.min_max), 
                         jpeg=self.jpeg, jpeg_quality=self.settings.jpeg_quality)
                         for raw, processor in zip(self.processors, images)]


