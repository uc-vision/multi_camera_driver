from abc import ABCMeta, abstractmethod, abstractproperty
from typing import Any, Dict

from spinnaker_camera_driver_helpers.camera_set import CameraSet, CameraSettings
from spinnaker_camera_driver_helpers.image_settings import ImageSettings, PublisherSettings
from .image_processor import EncoderError
import rospy
import PySpin

from queue import Queue
from threading import Thread

from .publisher import CameraPublisher
from py_structs import struct


class IncompleteImageError(Exception):
  def __init__(self, status):
    self.status = status
    
  def __str__(self):
    return f"Incomplete image: {self.status}"


def spinnaker_image(image, camera_info):
    if image.IsIncomplete():
      status = image.GetImageStatus()
      image.Release()          
      raise IncompleteImageError(status)
      

    image_data = image.GetNDArray()
    image_data.setflags(write=True)  # Suppress pytorch warning about non-writable array (we don't write to it.)

    image_info = struct(
      image_data = image_data,
      timestamp = rospy.Time.from_sec(image.GetTimeStamp() / 1e9 + camera_info.time_offset_sec),
      seq = image.GetFrameID()
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


class ImageHandler(BaseHandler):
  def __init__(self, cameras:CameraSet, settings:ImageSettings):

    self.camera_names = cameras.camera_ids
    self.handlers = {
      k:  CameraHandler(
        CameraPublisher(k, PublisherSettings(settings, cameras.camera_settings[k])))
          for k in cameras.camera_ids
    }
    self.report_rate = rospy.Duration.from_sec(4.0)
    self.reset_recieved()
    
  def reset_recieved(self):
    self.recieved = {k:0 for k in self.camera_names}
    self.dropped = 0
    self.published = 0
    self.update = rospy.Time.now()

  def report_recieved(self):
    duration = rospy.Time.now() - self.update
    if duration > self.report_rate:
      if self.dropped > 0:
        rospy.logwarn(f"published {self.published}, dropped {self.dropped}, received {self.recieved} in {format_sec(duration)}")
      else:
        rospy.logdebug(f"published {self.published}, {self.recieved} in {format_sec(duration)}")
      self.reset_recieved()

  def publish(self, image, camera_name, camera_info):
    self.handlers[camera_name].publish(image, camera_info)

  def update_camera(self, k:str, info:CameraSettings):
    self.handlers[k].update_camera(info)

  def update_settings(self, settings:ImageSettings):
    for handler in self.handlers.values():
      if handler.update_settings(settings):
        return True

  def start(self):
    for handler in self.handlers.values():
      handler.start()  


  def stop(self):
    for handler in self.handlers.values():
      handler.stop()  

