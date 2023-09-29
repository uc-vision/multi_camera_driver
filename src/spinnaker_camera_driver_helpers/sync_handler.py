
from dataclasses import replace
import datetime
from statistics import mean
import time
from typing import Dict, List, Tuple
import rospy


from spinnaker_camera_driver_helpers.common import CameraImage, CameraSettings
from spinnaker_camera_driver_helpers.work_queue import WorkQueue


from pydispatch import Dispatcher

from .image_handler import IncompleteImageError, format_msec, format_sec, spinnaker_image, take_group

from beartype import beartype

import PySpin
from natsort import natsorted
import torch


class SyncHandler(Dispatcher):
  _events_ = ["on_frame", "on_dsync"]

  @beartype
  def __init__(self, camera_settings:Dict[str, CameraSettings], device:torch.device,
    timeout_msec=1000, sync_threshold_msec=10.0):

    self.camera_settings = camera_settings
    self.queue = WorkQueue("SyncHandler", run=self.process_image, max_size=len(camera_settings))

    self.timeout = rospy.Duration.from_sec(timeout_msec / 1000.0)
    self.sync_threshold = rospy.Duration.from_sec(sync_threshold_msec / 1000.0)

    self.report_rate = rospy.Duration.from_sec(4.0)
    self.frame_queue:List[CameraImage] = []
    self.trigger_queue:List[Tuple[int, datetime.datetime]] = []

    self.camera_ids = natsorted(camera_settings.keys())
    self.camera_offsets = {k: rospy.Duration(0.0) for k in self.camera_ids}

    self.start_time = rospy.Time.now()

    self.running_clock_drift = 0.0
    self.reset_recieved()

    self.queue.start()
    self.device = device
    
  def reset_recieved(self):
    self.recieved = 0 
    self.dropped = {k:0 for k in self.camera_ids}
    self.published = 0

    self.update = rospy.Time.now()


  def report_recieved(self):
    now = rospy.Time.now()
    duration = now - self.update
    if duration > self.report_rate:
      message = f"published {self.published} ({self.published / duration.to_sec() : .2f} fps), {self.recieved} in {format_sec(duration)}"

      if sum(self.dropped.values()) > 0:
       rospy.logwarn(f"dropped {self.dropped}, {message}")
      else:
       rospy.logdebug(message)

      time_offsets = [ f"{k}={format_msec(self.camera_offsets[k])}" 
                      for k in sorted(self.camera_offsets.keys()) ]
      
      total = time.strftime("%H:%M:%S", time.gmtime((now - self.start_time).to_sec()))

      rospy.logdebug(f"camera offsets: {', '.join(time_offsets)}, clock offset: {1000.0 * self.running_clock_drift:.2f}ms, running time {total}")

      self.reset_recieved()

  @beartype
  def publish(self, camera_image_pair:Tuple[str, PySpin.ImagePtr]):
      self.queue.enqueue( camera_image_pair )

  @beartype
  def trigger_time(self, trigger_meta:Tuple[int, datetime.datetime]):

    highest_trigger_id = -1
    if len(self.trigger_queue) > 0:
      highest_trigger_id = max([trigger[0] for trigger in self.trigger_queue])

    if trigger_meta[0] < highest_trigger_id:
      rospy.logwarn(f"Trigger has lower id ({trigger_meta[0]}) than previous ({highest_trigger_id}) - flushing trigger queue")
    
    self.trigger_queue.append(trigger_meta)
    self.trigger_queue.sort(key=lambda r: r[0])
    if len(self.trigger_queue) > 10:
      self.trigger_queue = self.trigger_queue[-10:] 

  def add_frame(self, image_info):
    self.recieved += 1
    self.frame_queue.append(image_info)
    self.frame_queue.sort(key=lambda r: r.timestamp)

    timeout_time = self.frame_queue[-1].timestamp - self.timeout
    while(len(self.frame_queue) > 0 and self.frame_queue[0].timestamp < timeout_time):
      rospy.logwarn(f"dropping frame from {self.frame_queue[0].camera_name} because it is too old, {self.frame_queue[0].timestamp} < {timeout_time} ({(timeout_time - self.frame_queue[0].timestamp)/1e6}ms)")
      frame:CameraImage = self.frame_queue.pop(0)
      self.dropped[frame.camera_name] += 1

  def update_offsets(self, group):
    times = {k:frame.timestamp for k, frame in group.items()}
    mean_time = rospy.Time.from_sec( mean([time.to_sec() for time in times.values()]) )
    
    self.camera_offsets = {k: self.camera_offsets[k] + (time - mean_time) 
      for k, time in times.items()}

    
  def try_publish(self):
    found = take_group(self.frame_queue, self.sync_threshold, len(self.camera_ids))
    if found is not None:
      group, timestamp, self.frame_queue = found

      settings = list(self.camera_settings.values())[0]
      if settings.master_id is not None:
        seq_num = group[settings.master_id].seq
        rospy.loginfo(f"Master frame {seq_num} at {timestamp}")
        for i in self.trigger_queue:
          if i[0] == seq_num:
            rospy.loginfo(f"Trigger time {i[1]}")

            group = {k:replace(image, utc_time=i[1])
                    for k, image in group.items()}

      self.published += 1

      # Update offsets *before* setting timestamps equal
      self.update_offsets(group)  
              
      # Set timestamps to be equal
      group = {k:replace(image, timestamp=timestamp)
                    for k, image in group.items()}
      
      clock_time = min([image.clock_time for image in group.values()])
      diff = clock_time - timestamp

      t = 0.01
      self.running_clock_drift = self.running_clock_drift * t + diff.to_sec() * (1 - t)

      self.emit("on_frame", group)

  def process_image(self, camera_image_pair:Tuple[str, PySpin.ImagePtr]):
    camera_name, image = camera_image_pair

    try:
      offset = self.camera_settings[camera_name].time_offset_sec - self.camera_offsets[camera_name]
      image_info = spinnaker_image(camera_name, image, time_offset_sec=offset, device=self.device)


      self.add_frame(image_info)
      self.try_publish()
        
    except (PySpin.SpinnakerException, IncompleteImageError) as e:
      rospy.logwarn(f"{camera_name}: {e}")


  def stop(self):
    self.queue.stop()
    self.frame_queue = []

