
import rospy

from queue import Queue
from threading import Thread
from typing import Any, Dict, Optional, Tuple
from camera_geometry_ros.lazy_publisher import LazyPublisher

from sensor_msgs.msg import CameraInfo, CompressedImage, Image
from std_msgs.msg import Header

from cv_bridge import CvBridge
from dataclasses import dataclass, field

from sensor_msgs.msg import CameraInfo
from camera_geometry_ros.conversions import camera_info_msg


from .image_processor import image_backend
from py_structs import struct

@dataclass
class ImageSettings:
  cache_path : str 
  image_size : Tuple[int, int] = field(default_factory=lambda: (0, 0))
  encoding : str = 'bayer_bggr8' 
  device : str = 'cuda:0'
  queue_size : int = 4

  preview_size : int = 400
  quality : int = 90
  image_backend : str = 'turbo_jpeg'

  resize_width: Optional[int] = None
  sharpen: float = 0.0
  
class InvalidOption(Exception):
  pass


class CameraPublisher():
  def __init__(self, camera_name, settings, calibration=None):
    
    self.camera_name = camera_name
    self.settings = settings

    self.backend = image_backend(settings.image_backend)
    self.image_processor = None

    self.calibration = calibration

    self.queue = Queue(1)
    self.worker = None
    
    bridge = CvBridge()

    topics = {
        "image_raw"        : (Image, lambda data: bridge.cv2_to_imgmsg(data.image.raw, encoding=settings.encoding)),
        "compressed"       : (CompressedImage, lambda data: CompressedImage(data = data.image.compressed, format = "jpeg")), 
        "preview/compressed" :  (CompressedImage, lambda data: CompressedImage(data = data.image.preview, format = "jpeg")),
        "camera_info" : (CameraInfo, lambda data: data.camera_info)
    }

    self.publisher = LazyPublisher(topics, self.register, name=self.camera_name)


  def register(self):
    return []     # Here's where the lazy subscriber subscribes to it's inputs (none for this)


  def update_calibration(self, camera):
    self.calibration = camera

  @property
  def camera_info(self):
    width, height = self.settings.image_size

    if self.calibration is not None:
      calibration = self.calibration.resize_image(width, height)
      return camera_info_msg(calibration)
    else:
      return CameraInfo(width = width, height = height)


  def set_option(self, key:str, value:Any):
    if key == "image_size":

      if not (self.worker is None):
        raise InvalidOption("image size cannot be changed while running")
      self.settings.image_size = value
      
    elif key == "preview_size" and value > 0:
      self.settings.preview_size = value
    elif key == "quality":
      if (1 < value <= 100):
        self.settings.jpeg_quality = value
      else:
        raise InvalidOption(f"Invalid quality value {value}, should be 0 < quality <= 100")
    else:
      raise InvalidOption(f"Unknown or invalid option {key}={value}")
    

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
        self.publisher.publish(data=data, header=header)
        item = self.queue.get()

  def stop(self):
    rospy.loginfo(f"Waiting for {self.camera_name}: thread {self.worker}")
    
    self.queue.put(None)
    self.worker.join()
    print(f"Done {self.camera_name}: thread {self.worker}")

    self.worker = None

  def start(self):
    assert self.worker is None

    # Lazily create in case of image size changes
    self.image_processor = self.backend(self.settings)

    self.worker = Thread(target = self.publish_worker)
    self.worker.start()



    
    


