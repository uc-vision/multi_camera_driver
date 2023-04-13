
from typing import Callable, Optional
from beartype import beartype
from cached_property import cached_property

from nvjpeg_torch import Jpeg, JpegException

from spinnaker_camera_driver_helpers.common import CameraImage, EncoderError
from camera_geometry_ros.conversions import camera_info_msg
from camera_geometry import Camera
from sensor_msgs.msg import CameraInfo

import torch


class ImageOutputs(object):

  @beartype
  def __init__(self, camera_image:CameraImage, rgb:torch.Tensor, preview:torch.Tensor, 
               encode:Callable, calibration:Optional[Camera]=None):
    
    self.calibration = calibration
    
    self.raw = camera_image
    self.rgb = rgb
    self.preview = preview
    self.encode = encode


  @cached_property
  def compressed(self):
    return self.encode(self.rgb)

  @cached_property
  def compressed_preview(self):
      return self.encode(self.preview)
  
  @property
  def camera_name(self) -> str:
    return self.raw.camera_name

  @property
  def camera_info(self) -> CameraInfo:
    height, width, _ = self.rgb.shape

    if self.calibration is not None:  
      calibration = self.calibration.resize_image( (width, height) )
      return camera_info_msg(calibration)

    else:
      return CameraInfo(width = width, height = height)



