from __future__ import annotations
from typing import List

from camera_geometry_ros.lazy_publisher import LazyPublisher
from sensor_msgs.msg import CompressedImage, Image, CameraInfo

from std_msgs.msg import Header

from cv_bridge import CvBridge
from py_structs import struct

from spinnaker_camera_driver_helpers.image_processor.outputs import ImageOutputs
from spinnaker_camera_driver_helpers.work_queue import WorkQueue


class CameraPublisher():

  def __init__(self, camera_name:str):    
    self.camera_name = camera_name
    self.queue = WorkQueue(name=f"CameraPublisher({camera_name})", run=self.publish_worker, max_size=1)
    self.queue.start()

    bridge = CvBridge()
    topics = {
        "color"        : (Image, lambda data: bridge.cv2_to_imgmsg(data.rgb.cpu().numpy(), encoding="rgb8")),
        "color/compressed"       : (CompressedImage, lambda data: CompressedImage(data = data.compressed, format = "jpeg")), 
        "color/preview/compressed" :  (CompressedImage, lambda data: CompressedImage(data = data.compressed_preview, format = "jpeg")),
        "camera_info" : (CameraInfo, lambda data: data.camera_info)
    }

    self.publisher = LazyPublisher(topics, self.register, name=self.camera_name)

  def register(self):
    return []     # Here's where the lazy subscriber subscribes to it's inputs (we have no other ROS based inputs)


  def publish(self, image:ImageOutputs):
      return self.queue.enqueue( image )


  def publish_worker(self, image):
      header = Header(frame_id=image.raw.camera_name, stamp=image.raw.timestamp, seq=image.raw.seq)
      self.publisher.publish(data=image, header=header)

      del image


  def stop(self):
    self.queue.stop()

      

class FramePublisher():
     
    def __init__(self, camera_names:List[str]):
      self.publishers = {camera:CameraPublisher(camera) for camera in camera_names}
  
    def publish(self, images:List[ImageOutputs]):
      for image in images:
        self.publishers[image.camera_name].publish(image)
  

    def stop(self):
      for camera in self.publishers.values():
        camera.stop()