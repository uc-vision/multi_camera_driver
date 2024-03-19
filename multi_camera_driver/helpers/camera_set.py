from dataclasses import replace
from traceback import format_exc
from beartype.typing import Dict, List, Optional

import PySpin
import rospy2 as rospy
from camera_geometry import Camera
from multi_camera_driver.helpers import camera_setters

from multi_camera_driver.helpers.common import \
    CameraSettings, camera_encodings

from pydispatch import Dispatcher

from . import spinnaker_helpers
from .trigger_reporter import TriggerReporter


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
  _events_ = ["on_settings", "on_image", "on_trigger_time"]
  

  def __init__(self, camera_serials:Dict[str, str], 
      camera_settings:List[Dict],
      interface_settings:List[Dict],
      master_id:Optional[str]=None,
      trigger_reporter:Optional[str]=None,
      calibration:Dict[str, Camera]={}):

    self.started = False

    if not (master_id is None or master_id in camera_serials):
      raise ValueError(f"Master camera '{master_id}' not in camera_serials {camera_serials}")
    
    self.master_id = master_id

    self.trigger_reporter = None
    if trigger_reporter is not None:
      rospy.loginfo(f"Starting trigger reporter {trigger_reporter}")
      self.trigger_reporter = TriggerReporter(trigger_reporter)

      def trigger_callback(count, utc):
        self.emit("on_trigger_time", (count, utc))       
      self.trigger_reporter.trigger_callback = trigger_callback

      self.trigger_reporter.start()

    self.camera_serials = camera_serials

    # camera_name -> camera
    self.camera_dict = spinnaker_helpers.wait_for_cameras(camera_serials)  
    rospy.loginfo(f"Initialising cameras: {camera_serials}")
    rospy.loginfo(
        f"Triggering: {'Disabled' if self.master_id is None else 'Enabled'}")

    self.calibration = calibration
    self.camera_settings = {k: self.init_camera(camera, k, camera_settings)
                        for k, camera in self.camera_dict.items()}
    
    self._handler_dict = None
    rospy.loginfo(f"{len(self.camera_dict)} Cameras initialised")

  @property
  def camera_ids(self):
    return list(self.camera_dict.keys())


  def check_camera_settings(self):
    modified = False
    for k, camera in self.camera_dict.items():
      settings = self._camera_update(camera, self.camera_settings[k])
      if settings != self.camera_settings[k]:
        self.camera_settings[k] = settings
        modified = True
  
    if modified:
      self.emit("on_settings", self.camera_settings)

    return modified
  
  def update_calibration(self, calibration):
    self.calibration = calibration
    camera_settings = {k:replace(camera, calibration=calibration.get(k, None)) 
                       for k, camera in self.camera_settings.items()}
    self.emit("on_settings", camera_settings)

  @property
  def master(self):
    if self.master_id is not None:
      return self.camera_dict[self.master_id]

  def camera_properties(self):
    return list(camera_setters.property_setters.keys())
  

  def set_property(self, config:dict, key, value):
    if not key in camera_setters.property_setters:
        return False
    
    setter = camera_setters.property_setters[key]

    try:
      rospy.loginfo(f"CameraSet: set_property {key}: {value}")
      for k, camera in self.camera_dict.items():
        config = {**self.camera_settings[k].settings, key: value}
        setter(camera, config, self.camera_settings[k])

        self.camera_settings[k].settings = config

    except PySpin.SpinnakerException as e:
      rospy.logwarn(f"set_property: {key} {value} {e} ")
      return False
    except spinnaker_helpers.NodeException as e:
      rospy.logwarn(f"set_property: {key} {value} {e} ")
      return False

    return True

  def start(self):
    assert not self.started
    rospy.loginfo("Begin acquisition")

    if self.trigger_reporter is not None:
      self.trigger_reporter.reset_count()

    # Start acquisition on slave cameras first
    for k, camera in self.camera_dict.items():
      if k != self.master_id:
        camera.BeginAcquisition()
        rospy.loginfo(f"starting acquisition {k}..")
        assert spinnaker_helpers.validate_streaming(camera),\
            f"Camera {k} did not begin streaming"
    self.camera_dict[self.master_id].BeginAcquisition()
    assert spinnaker_helpers.validate_streaming(self.camera_dict[self.master_id]),"Master Camera did not begin streaming"

    self.started = True

  def stop(self):
    if self.started:

      try:
        self.camera_dict[self.master_id].EndAcquisition()
        assert spinnaker_helpers.validate_not_streaming(self.camera_dict[self.master_id]),f'Not wanting to stop Master'
      except PySpin.SpinnakerException as e:
        rospy.logwarn("warning: " + str(e))

      for k, camera in self.camera_dict.items():
        if k != self.master_id:
          try:
            rospy.loginfo(f"Ending acquisition {k}..")
            camera.EndAcquisition()
            assert spinnaker_helpers.validate_not_streaming(camera),f'Not wanting to stop {k}'

          except PySpin.SpinnakerException as e:
            rospy.logwarn("warning: " + str(e))

      rospy.loginfo("Stop - done.")
      self.started = False

  def trigger(self):
    assert self.master_id is not None
    try:
      spinnaker_helpers.trigger(self.master)
    except PySpin.SpinnakerException as e:
      rospy.logerr("Error triggering: " + str(e))


  def _camera_update(self, camera, info:CameraSettings):

    if not self.started:
      time_offset_sec=rospy.Duration.from_sec(spinnaker_helpers.camera_time_offset(camera))
    else:
      time_offset_sec = info.time_offset_sec

    max_framerate = spinnaker_helpers.get_framerate_info(camera)

    return replace(info,
        image_size = spinnaker_helpers.get_image_size(camera),
        time_offset_sec = time_offset_sec,
        framerate = max_framerate
      )


  def _camera_info(self, camera_name, camera):
    encoding = spinnaker_helpers.get_camera_encoding(camera)
    if encoding not in camera_encodings:
      raise ValueError(f"Unsupported encoding {encoding}, options are: {list(camera_encodings.keys())}")

    framerate = spinnaker_helpers.get_framerate_info(camera)

    return CameraSettings(
          name=camera_name,
          connection_speed=spinnaker_helpers.get_current_speed(camera),
          serial=str(spinnaker_helpers.get_camera_serial(camera)),
          time_offset_sec=rospy.Duration.from_sec(spinnaker_helpers.camera_time_offset(camera)),
          master_id=self.master_id,
          framerate = framerate,
          image_size = spinnaker_helpers.get_image_size(camera),
          encoding = camera_encodings[encoding],
          calibration=self.calibration.get(camera_name, None),
          settings=camera_setters.get_config(camera)

    )

  def init_camera(self, camera: PySpin.Camera, camera_name:str, camera_settings:List[Dict]):
    try:
      camera.Init()
      assert spinnaker_helpers.validate_init(camera),\
          f"Camera {camera_name} did not initialise"

      spinnaker_helpers.set_camera_settings(camera, camera_settings)

      encoding = spinnaker_helpers.get_camera_encoding(camera)
      if encoding not in camera_encodings:
        raise ValueError(f"Unsupported encoding {encoding}, options are: {list(camera_encodings.keys())}")

      if self.master_id == camera_name:
        spinnaker_helpers.trigger_master(camera, True)
      else:
        spinnaker_helpers.trigger_slave(camera)

      info = self._camera_info(camera_name, camera)
      print(info)
      return info
    
    except PySpin.SpinnakerException as e:
      rospy.logerr(f"Could not initialise camera {camera_name}: {str(e)}")

  def register_handlers(self):
    assert self._handler_dict is None, "Handlers already registered"

    def camera_handler(k):
      def on_image(image): 
        return self.emit("on_image", (k, image))
        
      return ImageEventHandler(on_image)
    self._handler_dict =  {k: camera_handler(k)
          for k in self.camera_dict.keys()}

    for k, handler in self._handler_dict.items():
      camera = self.camera_dict[k]
      camera.RegisterEventHandler(handler)    


  def unregister_handlers(self):
    if self._handler_dict is not None:
      for k, handler in self._handler_dict.items():
        camera = self.camera_dict[k]
        camera.UnregisterEventHandler(handler)

    self._handler_dict = None
  

  def cleanup(self):
    rospy.loginfo("Cleanup cameras")
    self.unregister_handlers()
    del self._handler_dict

    for camera in self.camera_dict.values():
      spinnaker_helpers.load_defaults(camera)
      camera.DeInit()
      
    del camera
    del self.camera_dict

    if self.trigger_reporter is not None:
      self.trigger_reporter.stop()
      rospy.loginfo("Trigger monitoring stopped")


