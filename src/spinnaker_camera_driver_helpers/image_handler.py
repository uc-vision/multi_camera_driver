import rospy

from queue import Queue
from threading import Thread

from .publisher import CameraPublisher, ImageSettings


class CameraHandler(object):
    def __init__(self, publisher, queue_size=2):
      self.publisher = publisher

      self.queue = Queue(queue_size)
      self.thread = None


    def publish(self, image, camera_info):
        self.queue.put( (image, camera_info) )


    def worker(self):
      item = self.queue.get()
      while item is not None:
        image, camera_info = item
        self.process_image(image, camera_info)
        item = self.queue.get()

      self.publisher.stop()


    def process_image(self, image, camera_info):       
        if image.IsIncomplete():
            rospy.logerr('Image incomplete, status: %d' % image.GetImageStatus())
        else:

            self.publisher.publish(
              image_data = image.GetNDArray(),
              timestamp = rospy.Time.from_sec(image.GetTimeStamp() / 1e9 + camera_info.time_offset_sec),
              seq = image.GetFrameID()
            )

            image.Release()            

    def set_option(self, key, value):
        self.publisher.set_option(key, value)

    def stop(self):
        if self.thread is not None:
          self.queue.put(None)
          rospy.loginfo("Waiting for publisher thread...")

          self.thread.join()
          
        self.thread = None

    def start(self):
        self.publisher.start()        

        self.thread = Thread(target=self.worker)        
        self.thread.start()


    def set_option(self, key, value):
        self.publisher.set_option(key, value)



class CameraSet(object):
  def __init__(self, camera_names, settings=ImageSettings()):
    self.camera_names = camera_names
    self.publishers = {
      k:  CameraHandler(CameraPublisher(k, settings)) for k in camera_names
    }


  def publish(self, image, camera_name, camera_info):
    self.publishers[camera_name].publish(image, camera_info)


  def set_option(self, key, value):
    for publisher in self.publishers.values():
      publisher.set_option(key, value)

  def start(self):
    for publisher in self.publishers.values():
      publisher.start()  


  def stop(self):
    for publisher in self.publishers.values():
      publisher.stop()  
