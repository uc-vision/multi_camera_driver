
from typing import Dict, List
import numpy as np
import torch

from pydispatch import Dispatcher
from beartype import beartype

from nvjpeg_torch import Jpeg

from spinnaker_camera_driver_helpers.image_handler import CameraImage
from spinnaker_camera_driver_helpers.common import CameraSettings, EncoderError
from spinnaker_camera_driver_helpers.image_processor.outputs import ImageOutputs
from spinnaker_camera_driver_helpers.image_processor.util import TiQueue, ema
from spinnaker_camera_driver_helpers.image_settings import ImageSettings
from spinnaker_camera_driver_helpers.work_queue import WorkQueue
from .camera_processor import CameraProcessor

from py_structs.torch import shape_info


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

    self.queue = WorkQueue("FrameProcessor", run=self.process_worker, max_size=1)
    self.queue.start()

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
  def process(self, images:Dict[str, CameraImage]):
    return self.queue.enqueue(images)

  def encode(self, image:torch.Tensor):
    print("encode", image.shape, image.dtype, image.device)
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

    images = TiQueue.run_sync(self.process_images, images)

    for image, preview, k in zip(*images, camera_images.keys()):
      print(k, shape_info(image), shape_info(preview))


    outputs = [ImageOutputs(camera_images[k], image, preview, encode=self.encode, calibration=self.cameras[k].calibration)
                    for image, preview, k in zip(*images, camera_images.keys())]


    self.emit("on_frame", outputs)
  
  @beartype
  def process_images(self, images:List[torch.Tensor]):

    for processor, image in zip(self.processors, images):
      processor.load_image(image)

    min_maxs = np.array([processor.min_max for processor in self.processors])
    min_maxs = np.array([min_maxs[:, 0].min(), min_maxs[:, 1].max()])

    self.min_max = ema(self.min_max, min_maxs, self.settings.ema_alpha)
    return processor.outputs(self.min_max)


  def stop(self):
    self.queue.stop()
