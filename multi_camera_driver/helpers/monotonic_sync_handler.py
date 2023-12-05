
from dataclasses import replace
import datetime
from statistics import mean
import time
from typing import Dict, List, Tuple, Optional
import rospy2 as rospy

from multi_camera_driver.helpers.common import CameraImage, CameraSettings
from multi_camera_driver.helpers.work_queue import WorkQueue


from pydispatch import Dispatcher

from .image_handler import IncompleteImageError, format_msec, format_sec, spinnaker_image#, take_group

from beartype import beartype

import PySpin
from natsort import natsorted
import torch


class SyncHandler(Dispatcher):
  _events_ = ["on_frame"]

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
    self.camera_frame_offsets = {k: 0 for k in self.camera_ids}

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
      
      total = time.strftime("%H:%M:%S", time.gmtime((now - self.start_time).to_sec()))

      self.reset_recieved()


  @beartype
  def publish(self, camera_image_pair:Tuple[str, PySpin.ImagePtr]):
    self.queue.enqueue( camera_image_pair )

  @beartype
  def trigger(self, trigger_meta:Tuple[int, datetime.datetime]):
    self.trigger_queue.append(trigger_meta)
    self.trigger_queue.sort(key=lambda r: r[0])

    highest_trigger_id = max([trigger[0] for trigger in self.trigger_queue])  
    min_allowed_trigger_id = highest_trigger_id - 4 

    while(len(self.trigger_queue) > 0 and self.trigger_queue[0][0] < min_allowed_trigger_id):
      rospy.logwarn(f"dropping trigger because it is too old, {self.trigger_queue[0][0]} < {min_allowed_trigger_id}")
      self.trigger_queue.pop(0)


  def add_frame(self, image_info):
    self.recieved += 1
    self.frame_queue.append(image_info)
    self.frame_queue.sort(key=lambda r: r.seq)

    highest_frame_id = max([frame.seq for frame in self.frame_queue])  
    min_allowed_frame_id = highest_frame_id - 4         

    while(len(self.frame_queue) > 0 and self.frame_queue[0].seq < min_allowed_frame_id):
      rospy.logwarn(f"dropping frame from {self.frame_queue[0].camera_name} because it is too old, {self.frame_queue[0].seq} < {min_allowed_frame_id}")
      frame:CameraImage = self.frame_queue.pop(0)
      self.dropped[frame.camera_name] += 1

  def get_trigger(self, trigger_id):
    for trigger in self.trigger_queue:
      if trigger[0] == trigger_id:
        return trigger
    return None
  
  def get_group(self, frame_id:int):
    """Returns the complete group of images with the given frame id and associated trigger"""
    group = {frame.camera_name:frame for frame in self.frame_queue if frame.seq == frame_id}
    trigger = self.get_trigger(frame_id)
    if len(group) == len(self.camera_ids):# and trigger is not None:
      return group, trigger
    return None
  
  def get_lowest_id_complete_group(self):
    """Returns the complete group of images with the lowest frame id and associated trigger"""
    min_frame_id = min([frame.seq for frame in self.frame_queue])
    max_frame_id = max([frame.seq for frame in self.frame_queue])
    for frame_id in range(min_frame_id, max_frame_id + 1):
      found = self.get_group(frame_id)
      if found is not None:
        return found
    return None
    
  def remove_images_with_frame_id(self, frame_id):
    self.frame_queue = [frame for frame in self.frame_queue if frame.seq != frame_id]

    
  def try_publish(self):
    found = self.get_lowest_id_complete_group()
    if found is not None:
      group, trigger = found
      self.published += 1

      self.remove_images_with_frame_id(list(group.values())[0].seq)
      if trigger is not None:
        self.trigger_queue.remove(trigger)

      #TODO this needs to be matched to trigger utc
      # group = {k:replace(image, timestamp=timestamp)
      #               for k, image in group.items()}

      self.emit("on_frame", group)

  def process_image(self, camera_image_pair:Tuple[str, PySpin.ImagePtr]):
    camera_name, image = camera_image_pair
    #rospy.loginfo("Processing image")

    try:
      chunk_data = image.GetChunkData()
      #rospy.loginfo(f"{camera_name} {image.GetTimeStamp()} {chunk_data.GetTimestamp()} {image.GetFrameID()} {chunk_data.GetFrameID()}")
      image_info = spinnaker_image(camera_name, image, time_offset_sec=rospy.Duration(0), device=self.device)


      self.add_frame(image_info)
      self.try_publish()
        
    except (PySpin.SpinnakerException, IncompleteImageError) as e:
      rospy.logwarn(f"{camera_name}: {e}")


  def stop(self):
    self.queue.stop()
    self.frame_queue = []

