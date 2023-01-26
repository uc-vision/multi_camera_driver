from __future__ import annotations

import rospy

from queue import Queue
from threading import Thread
from camera_geometry_ros.lazy_publisher import LazyPublisher

from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg import Header

from cv_bridge import CvBridge
import sensor_msgs.msg

from camera_geometry_ros.conversions import camera_info_msg
import numpy as np

from .image_settings import PublisherSettings

from .image_processor import image_backend
from py_structs import struct


class CameraPublisher():

  def __init__(self, camera_name:str, settings:PublisherSettings):
    
    self.camera_name = camera_name
    self.settings = settings

    self.backend = image_backend(settings.image.image_backend)
    self.image_processor = None


    self.queue = Queue(1)
    self.worker = None
    
    bridge = CvBridge()

    topics = {
        "image_raw"        : (Image, lambda data: bridge.cv2_to_imgmsg(data.image.raw, encoding=settings.camera.encoding.value)),
        "compressed"       : (CompressedImage, lambda data: CompressedImage(data = data.image.compressed, format = "jpeg")), 
        "preview/compressed" :  (CompressedImage, lambda data: CompressedImage(data = data.image.preview, format = "jpeg")),
        "camera_info" : (sensor_msgs.msg.CameraInfo, lambda data: data.camera_info)
    }

    self.publisher = LazyPublisher(topics, self.register, name=self.camera_name)


  def register(self):
    return []     # Here's where the lazy subscriber subscribes to it's inputs (we have no other ROS based inputs)

  @property
  def calibration(self):
    return self.settings.camera.calibration

  
  @property
  def camera_info(self):
    size = self.settings.camera.image_size
    width, height = size

    if self.calibration is not None:  
      calibration = self.calibration.resize_image( size )
      
      if self.settings.image.resize_width > 0:
        scale_factor = self.settings.image.resize_width / width
        calibration = calibration.scale_image(scale_factor)

      return camera_info_msg(calibration)

    else:
      return sensor_msgs.msg.CameraInfo(width = width, height = height)

  def update_settings(self, settings:PublisherSettings):
    if (settings.camera.image_size != self.settings.camera.image_size):
      return True
    
    if self.image_processor is not None:
      if self.image_processor.update_settings(settings):
        return True

    self.settings = settings
    return False


  def publish(self, image_data, timestamp, seq):
      data = struct(
        image = self.image_processor(image_data),
        camera_info = self.camera_info
      )

      header = Header(frame_id=self.camera_name, stamp=timestamp, seq=seq)
      return self.queue.put( (data, header) )


  def publish_worker(self):
      item = self.queue.get()
      while item is not None:
        data, header = item

        # If the image size has changed ignore old images in the queue
        h, w = data.image.raw.shape[:2]
        if self.settings.camera.image_size != (w, h):
          continue

        self.publisher.publish(data=data, header=header)
        item = self.queue.get()

  def stop(self):
    rospy.loginfo(f"Waiting for {self.camera_name}: thread {self.worker}")
    
    self.queue.put(None)
    self.worker.join()
    print(f"Done {self.camera_name}: thread {self.worker}")

    self.image_processor = None
    self.worker = None

  def start(self):
    assert self.worker is None

    # Lazily create in case of image size changes
    self.image_processor = self.backend(self.settings)

    self.worker = Thread(target = self.publish_worker)
    self.worker.start()



    
    


