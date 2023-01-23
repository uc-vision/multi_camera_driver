
from dataclasses import replace
from statistics import mean
from typing import Any, Dict, Tuple
import rospy

from queue import Queue
from threading import Thread

from spinnaker_camera_driver_helpers.camera_set import CameraSettings, CameraSet
from spinnaker_camera_driver_ros.msg import CameraStatus, CameraArrayStatus


from .image_settings import ImageSettings, PublisherSettings

from .publisher import CameraPublisher
from .image_handler import BaseHandler, IncompleteImageError, spinnaker_image, EncoderError

from camera_geometry import Camera

import PySpin
from natsort import natsorted

from .diagnostics import CameraDiagnosticUpdater

def format_msec(dt):
  return f"{dt.to_sec() * 1000.0:.2f}ms"

def format_sec(dt):
  return f"{dt.to_sec():.2f}ms"


def group_cameras(frame_group):
  cameras = {}
  for frame in frame_group:
    if frame.camera_name in cameras:
      dt = frame.timestamp - cameras[frame.camera_name].timestamp
      rospy.logwarn(f"Duplicate frame {next.camera_name} dt={format_msec(dt)}")
    else:
      cameras[frame.camera_name] = frame
  return cameras

def take_threshold(frame_queue, i, sync_threshold=2.0):
    j = i + 1
    while (j < len(frame_queue)):
      next = frame_queue[j]
      if next.timestamp - frame_queue[i].timestamp > sync_threshold:
        break
      j = j + 1
    return (i, j)

def take_group(frame_queue, sync_threshold, min_size):
  for i in range(0, len(frame_queue)):
      start, end = take_threshold(frame_queue, i, sync_threshold)
      cameras = group_cameras(frame_queue[start:end])
      if len(cameras) >= min_size:
        return frame_queue[start].timestamp, cameras, frame_queue[:start] + frame_queue[end:]


class SyncHandler(BaseHandler):
  def __init__(self, settings:ImageSettings, camera_set:CameraSet,
    timeout_msec=1000, sync_threshold_msec=10.0):

    self.camera_set = camera_set
    self.diagnostics = CameraDiagnosticUpdater(camera_set.camera_serials)

    self.publishers = {
      k:  CameraPublisher(k, PublisherSettings(settings, camera_set.camera_settings[k])) 
        for k in self.camera_set.camera_ids
    }

    self.image_settings = settings

    self.queue = Queue(len(camera_set.camera_ids))
    self.thread = None

    self.timeout = rospy.Duration.from_sec(timeout_msec / 1000.0)
    self.sync_threshold = rospy.Duration.from_sec(sync_threshold_msec / 1000.0)

    self.report_rate = rospy.Duration.from_sec(4.0)
    self.frame_queue = []

    self.camera_offsets = {k: rospy.Duration(0.0) for k in camera_set.camera_ids}
    self.reset_recieved()
    
  def reset_recieved(self):
    self.recieved = 0 
    self.dropped = 0
    self.published = 0

    self.update = rospy.Time.now()


  def report_recieved(self):
    duration = rospy.Time.now() - self.update
    if duration > self.report_rate:
      message = f"published {self.published} ({self.published / duration.to_sec() : .2f} fps), {self.recieved} in {format_sec(duration)}"

      if self.dropped > 0:
       rospy.logwarn(f"dropped {self.dropped}, {message}")
      else:
       rospy.logdebug(message)

      rospy.logdebug([(k, format_msec(self.camera_offsets[k])) 
        for k in natsorted(self.camera_set.camera_ids) ])
      self.diagnostics.reset()
      self.reset_recieved()


  def publish(self, image, camera_name):
      self.queue.put( (image, camera_name) )


  def worker(self):
    item = self.queue.get()
    while item is not None:
      image, camera_name = item
      self.process_image(image, camera_name)
      item = self.queue.get()
  

  def add_frame(self, image_info, camera_name):
    self.recieved += 1
    self.diagnostics.add_recieved(camera_name, 1)
    self.frame_queue.append(image_info)
    self.frame_queue.sort(key=lambda r: r.timestamp)

    timeout_time = rospy.Time.now() - self.timeout
    while(len(self.frame_queue) > 0 and self.frame_queue[0].timestamp < timeout_time):
      self.frame_queue.pop(0)
      self.dropped += 1
      self.diagnostics.add_dropped(camera_name, 1)


  def update_offsets(self, group):
    times = {k:frame.timestamp for k, frame in group.items()}
    mean_time = rospy.Time.from_sec( mean([time.to_sec() for time in times.values()]) )
        
    self.camera_offsets = {k: self.camera_offsets[k] + (time - mean_time) 
      for k, time in times.items()}

    
  def try_publish(self):
    found = take_group(self.frame_queue, self.sync_threshold, len(self.camera_set.camera_ids))
    if found is not None:
      timestamp, group, self.frame_queue = found
     
      for k, frame in group.items():
        self.publishers[k].publish(frame.image_data, timestamp, frame.seq)
      self.published += 1

      self.update_offsets(group)

  def process_image(self, image, camera_name):
    try:
      image_info = spinnaker_image(image, self.camera_set.camera_settings[camera_name])
      image_info = image_info.extend_(camera_name=camera_name, 
        timestamp = image_info.timestamp - self.camera_offsets[camera_name])

      self.add_frame(image_info, camera_name)
      self.try_publish()
        
    except (PySpin.SpinnakerException, IncompleteImageError) as e:
      rospy.logwarn(f"{camera_name}: {e}")

      
  def update_camera(self, k:str, camera:CameraSettings):
    settings = replace(self.publishers[k].settings, camera=camera)
    self.publishers[k].update_settings(settings)
    

  def update_settings(self, image_settings:ImageSettings):  
    for publisher in self.publishers.values():
      settings = replace(publisher.settings, image=image_settings)
      if publisher.update_settings(settings):
        return True

    self.settings = settings
    return False


  def stop(self):
    if self.thread is not None:
      self.queue.put(None)
      rospy.loginfo(f"Waiting for publisher thread {self.thread}")

      self.thread.join()
      rospy.loginfo(f"Done {self.thread}")
      for publisher in self.publishers.values():
        publisher.stop()  

      self.frame_queue = []
    self.thread = None

  def start(self):
      for publisher in self.publishers.values():
        publisher.start()   

      self.thread = Thread(target=self.worker)        
      self.thread.start()
