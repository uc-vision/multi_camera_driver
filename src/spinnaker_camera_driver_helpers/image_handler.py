import PySpin
from .publisher import AsyncPublisher, SpinnakerPublisher, CameraPublisher, ImageSettings

          

class CameraSet(object):
  def __init__(self, camera_names, settings=ImageSettings()):
    self.camera_names = camera_names
    self.publishers = {
      k: AsyncPublisher (
        SpinnakerPublisher(
          CameraPublisher(k, settings))
      ) for k in camera_names.items()
    }


  def publish(self, camera_name, camera_info, image):
    self.publishers[camera_name].publish( (image, camera_info) )


  def set_option(self, key, value):
    for publisher in self.publishers:
      publisher.set_option(key, value)

  def stop(self):
    for publisher in self.publishers:
      publisher.stop()  
