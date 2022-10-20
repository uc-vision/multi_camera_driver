
from os import sync
from statistics import mean
from typing import Any, Dict, Tuple
import rospy

from queue import Queue
from threading import Thread

from .publisher import CameraPublisher, ImageSettings
from .image_handler import spinnaker_image, EncoderError

import PySpin
import torch
from natsort import natsorted

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


class SyncHandler(object):
  def __init__(self, camera_names, settings : Dict[str, ImageSettings], 
    timeout_msec=1000, sync_threshold_msec=10.0, calibration={}):

    self.camera_names = camera_names
    self.publishers = {
      k:  CameraPublisher(k, settings[k], calibration.get(k, None)) 
        for k in camera_names
    }

    self.queue = Queue(len(camera_names))
    self.thread = None

    self.timeout = rospy.Duration.from_sec(timeout_msec / 1000.0)
    self.sync_threshold = rospy.Duration.from_sec(sync_threshold_msec / 1000.0)

    self.report_rate = rospy.Duration.from_sec(4.0)
    self.frame_queue = []

    self.camera_offsets = {k: rospy.Duration(0.0) for k in self.camera_names}
    self.reset_recieved()
    
  def reset_recieved(self):
    self.recieved = {k:0 for k in self.camera_names}
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
        for k in natsorted(self.camera_names) ])

      self.reset_recieved()

  def update_calibration(self, calibration):
    for k, publisher in self.publishers.items():
      publisher.update_calibration(calibration.get(k, None))

  def publish(self, image, camera_name, camera_info):
      self.queue.put( (image, camera_name, camera_info) )


  def worker(self):
    item = self.queue.get()
    while item is not None:
      image, camera_name, camera_info = item
      self.process_image(image, camera_name, camera_info)
      item = self.queue.get()
  

  def add_frame(self, image_info):
      self.frame_queue.append(image_info)
      self.frame_queue.sort(key=lambda r: r.timestamp)

      timeout_time = rospy.Time.now() - self.timeout
      while(len(self.frame_queue) > 0 and self.frame_queue[0].timestamp < timeout_time):
        self.frame_queue.pop(0)
        self.dropped += 1

        # rospy.logwarn(f"{info.camera_name} dropping frame {info.timestamp}")


  def update_offsets(self, group):
    times = {k:frame.timestamp for k, frame in group.items()}
    mean_time = rospy.Time.from_sec( mean([time.to_sec() for time in times.values()]) )
        
    self.camera_offsets = {k: self.camera_offsets[k] + (time - mean_time) 
      for k, time in times.items()}


    
  def try_publish(self):
    found = take_group(self.frame_queue, self.sync_threshold, len(self.camera_names))
    if found is not None:
      timestamp, group, self.frame_queue = found
     
      for k, frame in group.items():
        self.publishers[k].publish(frame.image_data, timestamp, frame.seq)
      self.published += 1

      self.update_offsets(group)

  def process_image(self, image, camera_name, camera_info):
    try:
      image_info = spinnaker_image(image, camera_info)
      image_info = image_info.extend_(camera_name=camera_name, timestamp = image_info.timestamp - self.camera_offsets[camera_name])
      if image is None:
        return

      self.recieved[camera_name] += 1

      self.add_frame(image_info)
      self.try_publish()
        
    except PySpin.SpinnakerException as e:
      rospy.logerr(e)



  def set_camera_options(self, k, options :  Dict[str, Any] ):
    assert k in self.publishers
    self.publishers[k].set_options(options)


  def set_options(self, options : Dict[str, Any]):
    for publisher in self.publishers.values():
      publisher.set_options(options)

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
