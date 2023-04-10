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

def group_cameras(frame_group):
  cameras = {}
  for frame in frame_group:
    if frame.camera_name in cameras:
      dt = frame.timestamp - cameras[frame.camera_name].timestamp
      rospy.logwarn(f"Duplicate frame {next.camera_name} dt={format_msec(dt)}")
    else:
      cameras[frame.camera_name] = frame
  return cameras

def take_threshold(frame_queue, i, sync_threshold=2.0):
    j = i + 1
    while (j < len(frame_queue)):
      next = frame_queue[j]
      if next.timestamp - frame_queue[i].timestamp > sync_threshold:
        break
      j = j + 1
    return (i, j)

def take_group(frame_queue, sync_threshold, min_size):
  for i in range(0, len(frame_queue)):
      start, end = take_threshold(frame_queue, i, sync_threshold)
      cameras = group_cameras(frame_queue[start:end])
      if len(cameras) >= min_size:
        return frame_queue[start].timestamp, cameras, frame_queue[:start] + frame_queue[end:]


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