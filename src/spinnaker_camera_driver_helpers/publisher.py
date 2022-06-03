

from queue import Queue
from threading import Thread
from camera_geometry_ros.lazy_publisher import LazyPublisher

from sensor_msgs.msg import CameraInfo, CompressedImage, Image
from std_msgs.msg import Header

from cv_bridge import CvBridge
from dataclasses import dataclass

from sensor_msgs.msg import CameraInfo
from camera_geometry_ros.conversions import camera_info_msg
from cached_property import cached_property

from .image_processor import image_backend


@dataclass
class ImageSettings:
  preview_size : int = 400
  encoding : str = 'bayer_bggr8' 
  device : str = 'cuda:0'
  queue_size : int = 4
  quality : int = 90
  image_backend : str = 'turbo_jpeg'


class CameraOutputs():
  def __init__(self, parent, image_raw):
    self.parent = parent
    self.image = self.parent.image_processor(image_raw)
    self.image_raw = image_raw


  @cached_property
  def camera_info(self):
    if self.parent.calibration is not None:
      calibration = self.parent.calibration.resize_image(
              (self.image_raw.shape[1], self.image_raw.shape[0]))
      return camera_info_msg(calibration)
    else:
      return CameraInfo()




class CameraPublisher():
  def __init__(self, camera_name, settings, calibration=None):
    
    self.camera_name = camera_name
    self.settings = settings

    processor = image_backend(settings.image_backend)
    self.image_processor = processor(settings)

    self.calibration = calibration

    self.queue = Queue(1)
    self.worker = Thread(target = self.publish_worker)
    
    bridge = CvBridge()

    topics = {
        "image_raw"        : (Image, lambda data: bridge.cv2_to_imgmsg(data.image.raw, encoding=settings.encoding)),
        "image_color"      : (Image, lambda data: bridge.cv2_to_imgmsg(data.image.color, encoding="bgr8")),
        "compressed"       : (CompressedImage, lambda data: CompressedImage(data = data.image.compressed, format = "jpeg")), 
        "preview/compressed" :  (CompressedImage, lambda data: CompressedImage(data = data.image.preview, format = "jpeg")),
        "camera_info" : (CameraInfo, lambda data: data.camera_info)
    }

    self.publisher = LazyPublisher(topics, self.register, name=self.camera_name)


  def register(self):
    return []     # Here's where the lazy subscriber subscribes to it's inputs (none for this)


  def update_calibration(self, camera):
    self.calibration = camera

  def set_option(self, option, value):
    if option == "preview_size":
      self.settings.preview_size = value
    elif option == "quality":
      assert value > 0 and value <= 100
      self.settings.quality = value
    else:
      assert False, f"unknown option {option}"


  def publish(self, image_data, timestamp, seq):
      data = CameraOutputs(self, image_data)
      header = Header(frame_id=self.camera_name, stamp=timestamp, seq=seq)
      return self.queue.put( (data, header) )


  def publish_worker(self):
      item = self.queue.get()
      while item is not None:
        data, header = item
        self.publisher.publish(data=data, header=header)
        item = self.queue.get()

  def stop(self):
    self.queue.put(None)
    self.worker.join()

  def start(self):
    self.worker.start()



    
    


