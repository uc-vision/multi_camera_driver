from __future__ import annotations

import rospy

from queue import Queue
from threading import Thread
from camera_geometry_ros.lazy_publisher import LazyPublisher

from sensor_msgs.msg import CompressedImage, Image
from spinnaker_camera_driver_helpers.image_handler import CameraImage
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


  def publish(self, image:CameraImage):
      return self.queue.put( image )


  def publish_worker(self):
      # Lazily create in case of image size changes
      self.image_processor = self.backend(self.settings)

      image:CameraImage = self.queue.get()
      while image is not None:
        # If the image size has changed ignore old images in the queue
        if self.settings.camera.image_size != image.image_size:
          rospy.loginfo_once(f"Dropping image: expected {self.settings.camera.image_size} got {image.image_size}")
          continue

        if self.settings.camera.encoding != image.encoding:
          rospy.logwarn_once(f"Dropping image: encoding inconsistent expected{self.settings.camera.encoding}, got {image.encoding}")
          continue
        

        header = Header(frame_id=image.camera_name, stamp=image.timestamp, seq=image.seq)
        outputs = struct(image=self.image_processor(image), camera_info=self.camera_info)

        self.publisher.publish(data=outputs, header=header)
        image = self.queue.get()

  def stop(self):
    rospy.loginfo(f"Waiting for {self.camera_name}: thread {self.worker}")
    
    self.queue.put(None)
    self.worker.join()
    print(f"Done {self.camera_name}: thread {self.worker}")

    self.image_processor = None
    self.worker = None

  def start(self):
    assert self.worker is None

    self.worker = Thread(target = self.publish_worker)
    self.worker.start()



    
    


