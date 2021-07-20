import PySpin
from .publisher import AsyncPublisher, SpinnakerPublisher, CameraPublisher, ImageSettings

          

class CameraSet(object):
  def __init__(self, camera_names, settings=ImageSettings()):
    self.camera_names = camera_names
    self.publishers = {
      k: AsyncPublisher (
        SpinnakerPublisher(
          CameraPublisher(k, settings))
      ) for k in camera_names
    }


  def publish(self, image, camera_name, camera_info):
    self.publishers[camera_name].publish( (image, camera_info) )


  def set_option(self, key, value):
    for publisher in self.publishers.values():
      publisher.set_option(key, value)

  def stop(self):
    for publisher in self.publishers.values():
      publisher.stop()  
