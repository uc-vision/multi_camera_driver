from dataclasses import dataclass, replace
from traceback import format_exc
from typing import Dict, List, Optional, Tuple

import PySpin
import rospy
from camera_geometry import Camera

from spinnaker_camera_driver_helpers.common import \
    CameraSettings, camera_encodings

from pydispatch import Dispatcher

from . import spinnaker_helpers


class ImageEventHandler(PySpin.ImageEventHandler):
  def __init__(self, on_image):
    super(ImageEventHandler, self).__init__()
    self.on_image = on_image

  def OnImageEvent(self, image):
    try:
      self.on_image(image)
    except:
      rospy.logerr(f"ImageEventHandler exception: {format_exc()}")



class CameraSet(Dispatcher):
  _events_ = ["on_camera_settings"]

  def __init__(self, camera_serials:Dict[str, str], 
      camera_settings:List[Dict], 
      master_id:Optional[str]=None,
      calibration:Dict[str, Camera]={}):

    self.started = False

    assert (master_id is None) or master_id in camera_serials
    self.master_id = camera_serials.get(master_id, None)
    self.camera_serials = camera_serials
    self.camera_dict = spinnaker_helpers.find_cameras(
        camera_serials)  # camera_name -> camera

    rospy.loginfo(f"Initialising cameras: {camera_serials}")
    rospy.loginfo(
        f"Triggering: {'Disabled' if self.master_id is None else 'Enabled'}")

    self.camera_settings = {k: self.init_camera(camera, k, camera_settings, calibration.get(k, None))
                        for k, camera in self.camera_dict.items()}
    
    self.register_handlers(self._camera_handlers)

    rospy.loginfo(f"{len(self.camera_dict)} Cameras initialised")

  @property
  def camera_ids(self):
    return list(self.camera_dict.keys())


  def check_image_sizes(self):

    image_sizes = self.get_image_sizes()    
    modified = any([camera.image_size != image_sizes[k] 
                    for k, camera in self.camera_settings.items()])
    
    if modified:
      self.camera_settings = {k: replace(self.camera_settings[k], image_size=image_size)
        for k, image_size in image_sizes.items()}

      self.emit("on_camera_settings", self.camera_settings)

    return modified

  @property
  def master(self):
    if self.master_id is not None:
      return self.camera_dict[self.master_id]

  def set_property(self, key, value, setter):
    try:
      rospy.logdebug(f"set_property {key}: {value}")
      for k, camera in self.camera_dict.items():
        setter(camera, value, self.camera_settings[k])

    except PySpin.SpinnakerException as e:
      rospy.logdebug(f"set_property: {key} {value} {e} ")
    except spinnaker_helpers.NodeException as e:
      rospy.logdebug(f"set_property: {key} {value} {e} ")

  def start(self):
    assert not self.started
    rospy.loginfo("Begin acquisition")

    for k, camera in self.camera_dict.items():
      camera.BeginAcquisition()
      assert spinnaker_helpers.validate_streaming(camera),\
          f"Camera {k} did not begin streaming"

    self.started = True

  def stop(self):
    if self.started:
      for k, camera in self.camera_dict.items():
        rospy.loginfo(f"Ending acquisition {k}..")
        camera.EndAcquisition()

      rospy.loginfo("Stop - done.")
      self.started = False


  def trigger(self):
    assert self.master_id is not None
    try:
      spinnaker_helpers.trigger(self.master)
    except PySpin.SpinnakerException as e:
      rospy.logerr("Error triggering: " + str(e))


  def get_image_sizes(self):
    return {k:spinnaker_helpers.get_image_size(camera)
      for k, camera in self.camera_dict.items()}


  def init_camera(self, camera: PySpin.Camera, camera_name:str, camera_settings:List[Dict], calibration:Optional[Camera]):
    try:
      camera.Init()
      assert spinnaker_helpers.validate_init(camera),\
          f"Camera {camera_name} did not initialise"

      is_master = camera_name == self.master_id
      spinnaker_helpers.set_camera_settings(camera, camera_settings)

      encoding = spinnaker_helpers.get_camera_encoding(camera)
      if encoding not in camera_encodings:
        raise ValueError(f"Unsupported encoding {encoding}, options are: {list(camera_encodings.keys())}")
      

      info = CameraSettings(
          name=camera_name,
          connection_speed=spinnaker_helpers.get_current_speed(camera),
          serial=spinnaker_helpers.get_camera_serial(camera),
          time_offset_sec=rospy.Duration.from_sec(spinnaker_helpers.camera_time_offset(camera)),
          is_master=is_master,
          is_free_running=is_master or self.master is None,
          image_size = spinnaker_helpers.get_image_size(camera),
          encoding = camera_encodings[encoding],
          calibration=calibration)

      rospy.loginfo(f"{camera_name}: {info}")

      if self.master_id is not None:
        spinnaker_helpers.trigger_master(camera, True)\
            if info.is_master else spinnaker_helpers.trigger_slave(camera)

      return info
    except PySpin.SpinnakerException as e:
      rospy.logerr(f"Could not initialise camera {camera_name}: {str(e)}")

  def _camera_handlers(self):
    def camera_handler(k):
      def on_image(image): 
        return self.emit("on_image", (k, image))
        
      return ImageEventHandler(on_image)
    handler_dict =  {k: camera_handler(k)
          for k in self.camera_dict.keys()}

    self.register_handlers(handler_dict)
    return handler_dict

  def register_handlers(self, handler_dict):
    for k, handler in handler_dict.items():
      camera = self.camera_dict[k]
      camera.RegisterEventHandler(handler)

  def unregister_handlers(self, handler_dict):
    for k, handler in handler_dict.items():
      camera = self.camera_dict[k]
      camera.UnregisterEventHandler(handler)
  

  def cleanup(self):
    rospy.loginfo("Cleanup cameras")

    for camera in self.camera_dict.values():
      spinnaker_helpers.load_defaults(camera)
      camera.DeInit()

    del self.camera_dict
    del camera


