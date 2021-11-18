from .image_processor import EncoderError
import rospy
import PySpin

from queue import Queue
from threading import Thread

from .publisher import CameraPublisher, ImageSettings
from structs.struct import struct

def spinnaker_image(image, camera_info):
    if image.IsIncomplete():
      rospy.logerr('Image incomplete, status: %d' % image.GetImageStatus())
      image.Release()          
      return None

    image_info = struct(
      image_data = image.GetNDArray(),
      timestamp = rospy.Time.from_sec(image.GetTimeStamp() / 1e9 + camera_info.time_offset_sec),
      seq = image.GetFrameID()
    )

    image.Release()    
    return image_info


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


    def set_option(self, key, value):
        self.publisher.set_option(key, value)

    def update_calibration(self, camera):
      return self.publisher.update_calibration(camera)

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



class ImageHandler(object):
  def __init__(self, camera_names, settings=ImageSettings(), calibration={}):

    self.camera_names = camera_names
    self.handlers = {
      k:  CameraHandler(
        CameraPublisher(k, settings, calibration.get(k, None))) 
          for k in camera_names
    }


  def publish(self, image, camera_name, camera_info):
    self.handlers[camera_name].publish(image, camera_info)

  def update_calibration(self, calibration):
    for k, handler in self.handlers.items():
      handler.update_calibration(calibration.get(k, None))

  def set_option(self, key, value):
    for handler in self.handlers.values():
      handler.set_option(key, value)

  def start(self):
    for handler in self.handlers.values():
      handler.start()  


  def stop(self):
    for handler in self.handlers.values():
      handler.stop()  

