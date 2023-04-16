
from typing import Dict, List
import numpy as np
import torch

from pydispatch import Dispatcher
from beartype import beartype

from nvjpeg_torch import Jpeg

from spinnaker_camera_driver_helpers.image_handler import CameraImage
from spinnaker_camera_driver_helpers.common import CameraSettings, EncoderError
from spinnaker_camera_driver_helpers.image_processor.outputs import ImageOutputs
from spinnaker_camera_driver_helpers.image_processor.util import TiQueue, ema, merge_metering
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

    print(len(cameras))


    self.processors = TiQueue.run_sync(self.init_processors, cameras)
    self.bounds = np.array([0, 1])
    self.metering = None

    self.intensity = 1.0

    self.queue = WorkQueue("FrameProcessor", run=self.process_worker, max_size=1)
    self.queue.start()

  def update_camera(self, k, camera):
    self.processors[k].update_camera(camera)

  def update_settings(self, settings:ImageSettings):

    for processor in self.processors:
      processor.update_settings(settings)


  @beartype
  def init_processors(self, cameras:Dict[str, CameraSettings]):

    return [CameraProcessor(name, self.settings, camera, device=self.settings.device) 
            for name, camera in cameras.items()]

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
    # images = [torch.from_numpy(image.image_data).to(device=self.settings.device) 
    #           for image in camera_images.values()]
  
    images = [image.image_data 
              for image in camera_images.values()]
  
    images = TiQueue.run_sync(self.process_images, images)


    outputs = [ImageOutputs(camera_images[k], image, preview, encode=self.encode, calibration=self.cameras[k].calibration)
                    for k, (image, preview) in zip(camera_images.keys(), images)]


    self.emit("on_frame", outputs)
  


  # @beartype
  def process_images(self, images:List[torch.Tensor]):

    for processor, image in zip(self.processors, images):
      processor.load_image(image)

    bounds = np.array([processor.bounds() for processor in self.processors])
    bounds = np.array([bounds[:, 0].min(), bounds[:, 1].max()])
    self.bounds = ema(self.bounds, bounds, self.settings.moving_average)

    camera_metering = [processor.metering(bounds) for processor in self.processors]
    self.metering = merge_metering(camera_metering, self.metering, self.settings.moving_average)
  
    return [processor.outputs(self.bounds, self.metering) 
            for processor in self.processors]


  def stop(self):
    self.queue.stop()
