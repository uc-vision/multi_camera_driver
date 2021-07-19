import rospy

from queue import Queue
from threading import Thread

import numpy as np
from structs.struct import struct

from camera_geometry_ros.lazy_publisher import LazyPublisher
from camera_geometry_ros.conversions import camera_info_msg

from sensor_msgs.msg import CameraInfo, CompressedImage, Image
from std_msgs.msg import Header

from nvjpeg_torch import Jpeg
import torch
from debayer import Debayer3x3

from cv_bridge import CvBridge
from dataclasses import dataclass
from cached_property import cached_property



@dataclass
class ImageSettings:
  preview_size : int = 400
  encoding : str = 'bayer_bggr8' 
  device : str = 'cuda:0'
  queue_size : int = 4
  quality : int = 90





class SpinnakerPublisher(object):
    def __init__(self, publisher, time_offset_sec):
        self.publisher = publisher
        self.time_offset_sec = time_offset_sec

    def publish(self, image):
        if image.IsIncomplete():
            rospy.logerr('Image incomplete, status: %d' % image.GetImageStatus())
        else:

            self.publisher.publish(
              image_data = image.GetNDArray(),
              timestamp = rospy.Time.from_sec(image.GetTimeStamp() / 1e9 + self.time_offset_sec),
              seq = image.GetFrameID()
            )

            image.Release()            

    def set_option(self, key, value):
        self.publisher.set_option(key, value)

    def stop(self):
        self.queue.put(None)

class AsyncPublisher(object):
    def __init__(self, publisher, queue_size=2):
        self.publisher = publisher

        self.queue = Queue(queue_size)
        self.thread = Thread(target=self.worker)
        self.thread.start()

    def worker(self):
      item = self.queue.get()
      while item is not None:
          self.publisher.publish(item)
          item = self.queue.get()

      self.publisher.stop()

    def publish(self, image):
        self.queue.put(image)

    def set_option(self, key, value):
        self.publisher.set_option(key, value)

    def stop(self):
        self.queue.put(None)



class CameraOutputs(object):
    def __init__(self, parent, image_raw):
        self.parent = parent
        self.image_raw = image_raw


    @property 
    def settings(self) -> ImageSettings:
      return self.parent.settings

    @cached_property
    def cuda_rgb(self):
      bayer = torch.from_numpy(self.image_raw).cuda()

      rgb = self.debayer(bayer.unsqueeze(1).to(dtype=torch.float16))
      return rgb.permute(0, 2, 3, 1).to(dtype=torch.uint8).squeeze(0)


    @cached_property
    def image_color(self):
      return self.cuda_rgb.cpu().numpy()

    @cached_property 
    def compressed(self):
      self.parent.encoder.encode(self.cuda_rgb, quality=self.quality)


    @cached_property 
    def preview(self):
      self.parent.encoder.encode(self.cuda_rgb, quality=self.quality)


    @cached_property
    def camera_info(self):
      if self.parent.calibration is not None:
        calibration = self.calibration.resize_image(
                (self.bayer_image.shape[1], self.bayer_image.shape[0]))
        return camera_info_msg(calibration)
      else:
        return CameraInfo()


class CameraPublisher():
  def __init__(self, camera_name, settings, calibration=None):
    
    self.camera_name = camera_name
    self.settings = settings

    self.encoder = Jpeg()
    self.debayer = Debayer3x3().to(dtype=torch.float16, device=self.device)
    self.calibration = calibration
    
    bridge = CvBridge()

    topics = dict(
        image_raw        = (Image, lambda data: bridge.cv2_to_imgmsg(data.image_raw, encoding=settings.encoding)),
        image_color      = (Image, lambda data: bridge.cv2_to_imgmsg(data.image_color, encoding="bgr8")),
        compressed = (CompressedImage, lambda data: CompressedImage(data = data.compressed, format = "jpeg")), 
        preview    =  (CompressedImage, lambda data: CompressedImage(data = data.preview, format = "jpeg")),
        camera_info = (CameraInfo, lambda data: data.camera_info)
    )

    self.publisher = LazyPublisher(topics, self.register)

  def register(self):
    return []     # Here's where the lazy subscriber subscribes to it's inputs (none for this)


  def set_quality(self, quality):
    self.quality = quality


  def set_option(self, option, value):
    if option == "preview_size":
      self.settings.preview_size = value
    elif option == "quality":
      assert value > 0 and value <= 100
      self.settings.quality = value
    else:
      assert False, f"unknown option {option}"


  def publish(self, image_data, timestamp, seq):
    return self.publisher.publish(
      data = CameraOutputs(self, image_data), 
      header = Header(frame_id=self.camera_name, stamp=timestamp, seq=seq)
    )


  def stop(self):
    pass


class CameraSet(object):
  def __init__(self, camera_properties, settings=ImageSettings()):
    self.camera_properties = camera_properties
    self.publishers = {
      k: AsyncPublisher (
        SpinnakerPublisher(
          CameraPublisher(k, settings), properties.time_offset_sec)
      ) for k, properties in camera_properties.items()
    }


  def publish(self, camera_name, image):
    self.publishers[camera_name].publish(image)


  def set_option(self, key, value):
    for publisher in self.publishers:
      publisher.set_option(key, value)

  def stop(self):
    for publisher in self.publishers:
      publisher.stop()  


    
    


