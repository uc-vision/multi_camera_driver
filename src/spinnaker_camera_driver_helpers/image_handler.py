
import numpy as np
from spinnaker_camera_driver_helpers.common import CameraImage, IncompleteImageError, from_pyspin
from dataclasses import replace
import rospy
import PySpin
import torch

import numba
from typing import List, Dict, Tuple, Optional


def spinnaker_image(camera_name:str, image:PySpin.Image, time_offset_sec:rospy.Duration, device:torch.device) -> CameraImage:
    if image.IsIncomplete():
      status = image.GetImageStatus()
      image.Release()          
      raise IncompleteImageError(status)

    clock_time = rospy.Time.now()
  
    image_data = image.GetData()
    image_data.setflags(write=True)  # Suppress pytorch warning about non-writable array (we don't write to it.)
    
    image_data = torch.from_numpy(image_data.view(
      np.uint8)).to(device, non_blocking=True)

    image_info = CameraImage(
      camera_name = camera_name,
      image_data = image_data.reshape(image.GetHeight(), -1),
      timestamp = rospy.Time.from_sec(image.GetTimeStamp() / 1e9) + time_offset_sec,
      clock_time = clock_time,

      seq = image.GetFrameID(),
      image_size = (image.GetWidth(), image.GetHeight()),
      encoding = from_pyspin(image.GetPixelFormat()),
    )

    image.Release()    
    return image_info


def format_msec(dt):
  return f"{dt.to_sec() * 1000.0:.2f}ms"

def format_sec(dt):
  return f"{dt.to_sec():.2f}s"

def group_cameras(frame_group:List[CameraImage]) -> Dict[str, CameraImage]:
  cameras = {}
  for frame in frame_group:
    if frame.camera_name in cameras:
      dt = frame.timestamp - cameras[frame.camera_name].timestamp
      rospy.logwarn(f"Duplicate frame {next.camera_name} dt={format_msec(dt)}")
    else:
      cameras[frame.camera_name] = frame
  return cameras

def take_threshold(frame_queue:List[CameraImage], i:int, sync_threshold:float=2.0) -> Tuple[int, int]:
    j = i + 1
    while (j < len(frame_queue)):
      next = frame_queue[j]
      if next.timestamp - frame_queue[i].timestamp > sync_threshold:
        break
      j = j + 1
    return (i, j)

def take_group(frame_queue:List[CameraImage], sync_threshold:float, min_size:int) -> Optional[Tuple[Dict[str, CameraImage], List[CameraImage]]]:
  if len(frame_queue) < min_size:
    return

  for i in range(0, len(frame_queue)):
      start, end = take_threshold(frame_queue, i, sync_threshold)
      images = group_cameras(frame_queue[start:end])
      if len(images) >= min_size:
        timestamp = frame_queue[start].timestamp
        return images, timestamp, frame_queue[:start] + frame_queue[end:]

  return None
