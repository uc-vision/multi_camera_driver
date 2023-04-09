from abc import ABCMeta, abstractmethod
import datetime
from typing import  Tuple
import numpy as np

from spinnaker_camera_driver_helpers.common import  CameraSettings
from spinnaker_camera_driver_helpers.common import CameraImage, IncompleteImageError, from_pyspin
from spinnaker_camera_driver_helpers.image_settings import ImageSettings
import rospy
import PySpin






def spinnaker_image(camera_name:str, image:PySpin.Image, time_offset_sec:rospy.Duration) -> CameraImage:
    if image.IsIncomplete():
      status = image.GetImageStatus()
      image.Release()          
      raise IncompleteImageError(status)
    
    image_data = image.GetData()
    image_data.setflags(write=True)  # Suppress pytorch warning about non-writable array (we don't write to it.)

    image_info = CameraImage(

      camera_name = camera_name,
      image_data = image_data,
      timestamp = rospy.Time.from_sec(image.GetTimeStamp() / 1e9) + time_offset_sec,
      seq = image.GetFrameID(),

      image_size = (image.GetWidth(), image.GetHeight()),
      encoding = from_pyspin(image.GetPixelFormat()),
    )

    image.Release()    
    return image_info


def format_msec(dt):
  return f"{dt.to_sec() * 1000.0:.2f}ms"

def format_sec(dt):
  return f"{dt.to_sec():.2f}ms"



class BaseHandler(metaclass=ABCMeta):

  @abstractmethod
  def reset_recieved(self):
    pass

  @abstractmethod
  def report_recieved(self):
    pass

  @abstractmethod
  def publish(self, image:PySpin.Image, camera_name:str, camera_info):
    pass

  @abstractmethod
  def update_camera(self, k:str, info:CameraSettings):
    pass

  @abstractmethod
  def update_settings(self, settings:ImageSettings) -> bool:
    pass


  @abstractmethod
  def start(self):
    pass

  @abstractmethod
  def stop(self):
    pass