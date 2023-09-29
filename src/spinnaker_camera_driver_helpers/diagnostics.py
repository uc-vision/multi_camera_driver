from beartype import beartype
import rospy
import diagnostic_updater
import diagnostic_msgs
from typing import List, Dict, Optional, Union
from spinnaker_camera_driver_helpers.camera_set import CameraSet

from spinnaker_camera_driver_helpers.common import CameraSettings

OK = diagnostic_msgs.msg.DiagnosticStatus.OK
WARN = diagnostic_msgs.msg.DiagnosticStatus.WARN
ERROR = diagnostic_msgs.msg.DiagnosticStatus.ERROR
STALE = diagnostic_msgs.msg.DiagnosticStatus.STALE

class CameraState(object):
  def __init__(self, 
            updater: diagnostic_updater.Updater, 
            camera_name: str, 
            camera_serial: Union[str, int],
            ideal_framerate: int,
            tolerance: float = 2,
            time_before_stale: float = 2):
    
    self.camera_name = camera_name
    self.camera_serial = str(camera_serial)
    self.time_before_stale = time_before_stale

    self.ideal_framerate = ideal_framerate
    self.tolerance = tolerance

    self.updated_time = rospy.Time()
    self.time_since_last_reset = rospy.Time().now()
    self._recieved = 0

    updater.add(camera_name, self.produce_diagnostics)

  @property
  def recieved(self):
    return self._recieved
  
  @recieved.setter
  def recieved(self, new_value):
    self._recieved = new_value
    self.updated_time = rospy.Time().now()
  
  
  def reset(self):
    self._recieved = 0
    self.time_since_last_reset = rospy.Time().now()

  def produce_diagnostics(self, stat):
    """ Check current state and report statistics """
    stat.hardware_id = self.camera_serial
    last_update = (rospy.Time.now() - self.updated_time).to_sec()

    if last_update > self.time_before_stale:
      stat.summary(ERROR, f'Haven\'t recieved update in {last_update} seconds')
      return stat
    
    last_reset = (rospy.Time.now() - self.time_since_last_reset).to_sec()
    lower_bound = (self.ideal_framerate * last_reset) - self.tolerance
    upper_bound = (self.ideal_framerate * last_reset) + self.tolerance
    if lower_bound > self.recieved or self.recieved > upper_bound:
      stat.summary(WARN, f'Recieved inadequate amount of frames: {self.recieved} instead of {self.ideal_framerate * last_reset}')
      return stat


    stat.summary(OK, f'Recieved {self.recieved} frames')
    return stat

class CameraDiagnosticUpdater:

  @beartype
  def __init__(self, camera_settings:Dict[str, CameraSettings], master_id:Optional[str],  tolerance: float = 2):
    """ Creates diagnostics tasks for each camera

    camera_serials is a dict composed of camera_serial->camera_name
    """
    self.updater = diagnostic_updater.Updater()
    self.updater.setHardwareID("cameras")  

    self.master_id = master_id    
    
    self.camera_states = {
      k: CameraState(self.updater, v.name, v.serial, 
                     ideal_framerate = v.framerate if v.is_master or self.master_id is None else camera_settings[self.master_id].framerate, 
                     tolerance=tolerance)

      for k, v in camera_settings.items()
    }

  def reset(self):
    self.updater.update()
    for v in self.camera_states.values():
      v.reset()

  def on_camera_info(self, camera_settings:Dict[str, CameraSettings]):
    for k, v in camera_settings.items():
      self.camera_states[k].ideal_framerate = v.framerate if v.is_master or self.master_id is None else camera_settings[self.master_id].framerate

  def on_image(self, image):
    camera_name, _ = image
    self.camera_states[camera_name].recieved += 1
    
  def update_framerate(self, new_framerate):
    for k, v in self.camera_states.items():
      v.ideal_framerate = new_framerate


