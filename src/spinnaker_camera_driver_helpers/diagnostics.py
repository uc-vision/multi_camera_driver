

import rospy
import diagnostic_updater
import diagnostic_msgs
from typing import List, Dict

OK = diagnostic_msgs.msg.DiagnosticStatus.OK
WARN = diagnostic_msgs.msg.DiagnosticStatus.WARN
ERROR = diagnostic_msgs.msg.DiagnosticStatus.ERROR
STALE = diagnostic_msgs.msg.DiagnosticStatus.STALE

class CameraState(object):
  def __init__(self, 
            updater: diagnostic_updater.Updater, 
            camera_name: str, 
            time_before_stale: int = 1):
    self.camera_name = camera_name
    self.time_before_stale = time_before_stale

    self.updated_time = rospy.Time(0)
    self.recieved = 0
    self.dropped = 0

    updater.add(camera_name, self.produce_diagnostics)
  
  def update(self, recieved, dropped):
    self.recieved = recieved
    self.dropped = dropped
    self.updated_time = rospy.Time.now()

  def produce_diagnostics(self, stat):
    """ Check current state and report statistics """
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
  def __init__(self, camera_ids: List[str]):
    self.updater = diagnostic_updater.Updater()
    self.updater.setHardwareID("none")
    self.camera_states = {
      k: CameraState(self.updater, k)
      for k in camera_ids
    }

    # Update stats every second
    def update_diagnostics(event):
      self.updater.update()
    rospy.Timer(rospy.Duration(1), update_diagnostics)

  def update(self, recieved: Dict[str, int], dropped: Dict[str, int]):
    """ Updates diagnostics of each cam 
    
    Given two dicts cam_id->recieved_frames, cam_id->dropped_frames
    """
    for cam_id, recieved_frames in recieved.items():
      dropped_frames = dropped[cam_id]
      self.camera_states[cam_id].update(recieved_frames, dropped_frames)
