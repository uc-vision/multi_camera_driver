

import rospy
import diagnostic_updater
import diagnostic_msgs
from typing import List, Dict, Union

OK = diagnostic_msgs.msg.DiagnosticStatus.OK
WARN = diagnostic_msgs.msg.DiagnosticStatus.WARN
ERROR = diagnostic_msgs.msg.DiagnosticStatus.ERROR
STALE = diagnostic_msgs.msg.DiagnosticStatus.STALE

class CameraState(object):
  def __init__(self, 
            updater: diagnostic_updater.Updater, 
            camera_name: str, 
            camera_serial: Union[str, int],
            time_before_stale: int = 3):
    self.camera_name = camera_name
    self.camera_serial = str(camera_serial)
    self.time_before_stale = time_before_stale

    self.updated_time = rospy.Time()
    self._recieved = 0
    self._dropped = 0

    updater.add(camera_name, self.produce_diagnostics)

  @property
  def recieved(self):
    return self._recieved
  
  @recieved.setter
  def recieved(self, new_value):
    self._recieved = new_value
    self.updated_time = rospy.Time.now()
  
  @property
  def dropped(self):
    return self._dropped
  
  @dropped.setter
  def dropped(self, new_value):
    self._dropped = new_value
    self.updated_time = rospy.Time.now()
  
  def reset(self):
    self._recieved = 0
    self._dropped = 0

  def produce_diagnostics(self, stat):
    """ Check current state and report statistics """
    stat.hardware_id = self.camera_serial
    last_update = (rospy.Time.now() - self.updated_time).to_sec()
    if last_update > self.time_before_stale:
      stat.summary(STALE, f'Haven\'t recieved update in {last_update} seconds')
      return stat
    if self.dropped > 0:
      stat.summary(ERROR, f'{self.camera_name} has dropped {self.dropped} frames')
    else:
      stat.summary(OK, f'Recieved {self.recieved} frames')
    return stat

class CameraDiagnosticUpdater:
  def __init__(self, camera_serials: Dict[str, str]):
    """ Creates diagnostics tasks for each camera

    camera_serials is a dict composed of camera_serial->camera_name
    """
    self.updater = diagnostic_updater.Updater()
    self.camera_states = {
      v: CameraState(self.updater, v, k)
      for k, v in sorted(camera_serials.items(), key=lambda x: x[1])
    }

  def reset(self):
    self.updater.update()
    for k, v in self.camera_states.items():
        v.reset()

  def add_recieved(self, camera_name, count):
    self.camera_states[camera_name].recieved += count

  def add_dropped(self, camera_name, count):
    self.camera_states[camera_name].dropped += count
