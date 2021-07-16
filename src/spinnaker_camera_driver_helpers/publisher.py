import rospy

from queue import Queue
from threading import Thread

import numpy as np
from structs.struct import struct

from spinnaker_camera_driver_helpers.common import publish_extrinsics
from camera_geometry_ros.image_publisher import RawPublisher

class ImagePublisher():
  def __init__(self, camera_names, calibrations={}, preview_size=400, threshold_msec=2.0):
    
    self.camera_names = camera_names
    self.calibrations = calibrations
    self.preview_size = preview_size

    self.threshold_msec=threshold_msec
    self.current_frame = {}

    self.quality = 90

    self.publish_times = []

    self.queue = Queue(len(camera_names))
    self.thread = Thread(target=self.process_thread, args=())

    self.thread.start()

    self.raw_publishers = {camera_name:RawPublisher(camera_name, "image_raw") for camera_name in camera_names}


  def stop(self):
    rospy.loginfo("Waiting for publisher thread...")
    self.queue.put(None)
    self.thread.join()

  def set_quality(self, quality):
    self.quality = quality

  
  def onImage(self, camera_name, image, time_offset_sec):
    assert camera_name in self.camera_names
    if image.IsIncomplete():
      rospy.logwarn('Image incomplete with image status %d ...' % image.GetImageStatus())

    timestamp = image.GetTimeStamp() / 1e9 + time_offset_sec 
    frame = struct(
      timestamp = timestamp,
      frame_id = image.GetFrameID(),
      image_data = image.GetNDArray()
    )

    self.queue.put( (camera_name, frame) )
    image.Release()


  def process_thread(self):
    item = self.queue.get()
    while item is not None: 
      (camera_name, frame) = item
      self.process_frame(camera_name, frame)
      item = self.queue.get()


  def process_frame(self, camera_name, frame):
    if camera_name in self.current_frame:
        self.publish_current()

    for k, current in self.current_frame.items():
      time_diff_msec = abs(frame.timestamp - current.timestamp) * 1000.0
      print(k, time_diff_msec)
      if time_diff_msec > self.threshold_msec:
        rospy.logwarn(f"Frame time for {camera_name} differs from {k} by {time_diff_msec} ms")


    self.current_frame[camera_name] = frame
    if len(self.camera_names) == len(self.current_frame):
      self.publish_current()    


  def publish_current(self):
    assert len(self.current_frame) > 0

    frame_times = [frame.timestamp for frame in self.current_frame.values()]
    latest = min(frame_times)

    missing = set(self.camera_names) - set(self.current_frame.keys())
    if len(missing) > 0:
      rospy.logwarn(f"Incomplete set of frames, missing: {missing}")


    if len(self.publish_times) > 0:
      while len(self.publish_times) > 0 and latest - self.publish_times[0] > 5.0:
        self.publish_times.pop(0)

      dt = (latest - self.publish_times[0])
      num_frames = len(self.publish_times)
      rospy.loginfo(f"{num_frames} {dt:.2f} {num_frames / dt:.2f} Hz")

    self.publish_times.append(latest)
    for k, publisher in self.raw_publishers.items():

      publisher.publish(self, self.current_frame[k], timestamp, cam_info=None)

    self.current_frame = {}


  def publish_extrinsics(self):
    found = {k:camera.parent_to_camera 
      for k, camera in  self.calibrations.items()}

    extrinsics = {alias:found.get(alias, np.eye(4))
      for alias in self.camera_names}

    namespace = rospy.get_namespace().strip('/')
    self.static_broadcaster = publish_extrinsics(namespace, extrinsics)    