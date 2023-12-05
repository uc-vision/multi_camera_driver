from __future__ import annotations
from typing import List

from camera_geometry_ros.lazy_publisher import LazyPublisher
from sensor_msgs.msg import CompressedImage, Image, CameraInfo, TimeReference

from std_msgs.msg import Header
import rospy2 as rospy

from cv_bridge import CvBridge
from py_structs import struct

from multi_camera_driver.helpers.image_processor.outputs import ImageOutputs
from multi_camera_driver.helpers.work_queue import WorkQueue

from pydispatch import Dispatcher

class CameraPublisher(Dispatcher):
  _events_ = ["subscribed", "unsubscribed"]

  def __init__(self, camera_name:str, namespace): 
   
    self.camera_name = camera_name
    self.queue = WorkQueue(name=f"CameraPublisher({camera_name})", run=self.publish_worker, max_size=1)
    self.queue.start()

    bridge = CvBridge()
    topics = {
        "color"        : (Image, lambda data: bridge.cv2_to_imgmsg(data.rgb.cpu().numpy(), encoding="rgb8")),
        "color/compressed"       : (CompressedImage, lambda data: CompressedImage(data = data.compressed, format = "jpeg")), 
        "color/preview/compressed" :  (CompressedImage, lambda data: CompressedImage(data = data.compressed_preview, format = "jpeg")),
        "camera_info" : (CameraInfo, lambda data: data.camera_info),
        "utc" : (TimeReference, lambda data: TimeReference( time_ref=rospy.Time.from_sec(data.raw.utc_time.timestamp()), source="utc time of capture"))
    }

    self.publisher = LazyPublisher(topics, name = f'{namespace}/{self.camera_name}', 
            on_register=self.register, on_unregister=self.unregister)

  def register(self):
    self.emit("subscribed", self.camera_name)
    return []     # Here's where the lazy subscriber returns subscriptions to it's inputs 

  def unregister(self, topics):
    self.emit("unsubscribed", self.camera_name)


  def publish(self, image:ImageOutputs):
      return self.queue.enqueue( image )


  def publish_worker(self, image):
      header = Header(frame_id=image.raw.camera_name, stamp=image.raw.timestamp, seq=image.raw.seq)
      self.publisher.publish(data=image, header=header)

      del image


  def stop(self):
    self.queue.stop()

      

class FramePublisher(Dispatcher):
    _events_ = ["any_subscribed", "all_unsubscribed"]

    def __init__(self, camera_names:List[str], namespace = ''):
      self.publishers = {camera:CameraPublisher(camera, namespace) 
            for camera in camera_names}
      
      self.cameras_subscribed = set()
    
    def subscribed(self, camera_name:str):
      first = len(self.cameras_subscribed) == 0
      self.cameras_subscribed.add(camera_name)
      if first:
        self.emit("any_subscribed")


    def unsubscribed(self, camera_name:str):
      self.cameras_subscribed.remove(camera_name)
      if len(self.cameras_subscribed) == 0:
        self.emit("all_unsubscribed")

    def publish(self, images:List[ImageOutputs]):
      for image in images:
        self.publishers[image.camera_name].publish(image)
  
    def stop(self):
      for camera in self.publishers.values():
        camera.stop()