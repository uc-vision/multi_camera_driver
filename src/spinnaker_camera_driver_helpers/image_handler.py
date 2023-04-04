from abc import ABCMeta, abstractmethod
import datetime
from typing import Any, Dict, Tuple
import numpy as np

from spinnaker_camera_driver_helpers.camera_set import CameraSet, CameraSettings
from spinnaker_camera_driver_helpers.image_processor.common import ImageEncoding, from_pyspin
from spinnaker_camera_driver_helpers.image_settings import ImageSettings, PublisherSettings
from .image_processor import EncoderError
import rospy
import PySpin

from queue import Queue
from threading import Thread

from dataclasses import dataclass


class IncompleteImageError(Exception):
  def __init__(self, status):
    self.status = status
    
  def __str__(self):
    return f"Incomplete image: {self.status}"


@dataclass
class CameraImage:
  camera_name: str
  image_data: np.ndarray
  timestamp: rospy.Time
  seq: int
  image_size: Tuple[int, int]
  encoding: ImageEncoding

  def __repr__(self):
    date = datetime.datetime.fromtimestamp(self.timestamp.to_sec())
    pretty_time = date.strftime("%H:%M:%S.%f")
    w, h = self.image_size

    return f"CameraImage({self.camera_name}, {w}x{h}, {self.image_data.shape[0]}:{str(self.image_data.dtype)}, {self.encoding.value}, {pretty_time}, {self.seq})"

  

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


class CameraHandler(object):
    def __init__(self, publisher, queue_size=2):
      self.publisher = publisher

      self.queue = Queue(queue_size)
      self.thread = None


    def publish(self, image, camera_info):
        self.queue.put( (image, camera_info) )


    def worker(self):
      self.publisher.start()        

      item = self.queue.get()
      while item is not None:
        image, camera_info = item

        try:
          image_info = spinnaker_image(image, camera_info) 
          if image_info is not None:
            self.publisher.publish(image_info.image_data, image_info.timestamp, image_info.seq)
        except PySpin.SpinnakerException as e:
          rospy.logerr(e)
        except EncoderError as e:
          rospy.logerr(e)

        item = self.queue.get()
      self.publisher.stop()

    def update_settings(self, settings:PublisherSettings):
        return self.publisher.update_settings(settings)

    def stop(self):
        if self.thread is not None:
          self.queue.put(None)
          rospy.loginfo(f"Waiting for publisher thread {self.thread}")

          self.thread.join()
          rospy.loginfo(f"Done {self.thread}")

        self.thread = None

    def start(self):

        self.thread = Thread(target=self.worker)        
        self.thread.start()

    def set_option(self, key, value):
        self.publisher.set_option(key, value)



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