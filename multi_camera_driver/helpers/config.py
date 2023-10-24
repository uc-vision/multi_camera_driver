from os import path
import os
import numpy as np
from py_structs import struct
import rospy2 as rospy
import shutil
import errno
import yaml
from typing import Dict, Union, NoReturn

import camera_geometry_ros.conversions as conversions
from camera_geometry.calib import import_rig
from camera_geometry import json

import traceback
import sys
import time

def load_config(config_file) -> Union[Dict, None]:
  if config_file is not None:
    with open(config_file) as config_file:
      return yaml.load(config_file, Loader=yaml.Loader)
  else:
    return None
    

def import_calibrations(calib_string, camera_names, tracking_frame):
  calib = json.load_string(calib_string)
  camera_calibrations = import_rig(calib)

  if set(camera_names) != set(camera_calibrations.keys()):
    rospy.logwarn(f"calibrations {set(camera_calibrations.keys())} don't match cameras {set(camera_names)}")

  tracking = None
  if 'tracking_rig' in calib and tracking_frame is not None:
    tracking = struct(frame=tracking_frame, transform=np.array(calib.tracking_rig))

  return struct(cameras = camera_calibrations, tracking = tracking)


def load_calibrations(calibration_file, camera_names, tracking_frame=None):
    rospy.loginfo(f"Loading calibrations from: {calibration_file}")

    camera_calibrations = struct(cameras = struct(), tracking = None)
    try:
      if calibration_file is not None:
          with open(calibration_file, 'rt') as file:
            calib_string = file.read()
            return import_calibrations(calib_string, camera_names, tracking_frame=tracking_frame)
    except FileNotFoundError:
        rospy.logwarn(f"Calibration file not found: {calibration_file}")


    return camera_calibrations

def camera_transforms(rig_frame, transforms, tracking=None):
    stamp = rospy.Time.now()

    parent_transform = []

    if tracking is not None:
      parent_transform = [conversions.transform_msg(
        tracking.transform, tracking.frame, rig_frame, stamp)]

    return [conversions.transform_msg(rig_to_cam, rig_frame, f"{rig_frame}/{child_id}", stamp)
            for child_id, rig_to_cam in transforms.items()] + parent_transform
    
def publish_extrinsics(broadcaster, calib, camera_names):
  found = {k:camera.parent_to_camera 
    for k, camera in  calib.cameras.items()}

  extrinsics = {alias:found.get(alias, np.eye(4))
    for alias in camera_names}

  rig_frame = rospy.get_param('rig_frame', '')
  msgs = camera_transforms(rig_frame, extrinsics, calib.tracking)   

  broadcaster.sendTransform(msgs)



def mkdir_p(path):
    try:
        os.makedirs(path)
    except OSError as exc:
        if exc.errno == errno.EEXIST and os.path.isdir(path):
            pass
        else:
            raise


def find_unique(base, filename):
  basename, ext = path.splitext(filename)
  modified = filename
  n = 0
  while(path.exists(path.join(base, modified))):
    n = n + 1
    modified = f"{basename}_{n:03}{ext}"
  return modified   


def write_calibration(calibration_filename, calibration):
  calib_path, filename = path.split(calibration_filename)

  if path.exists(filename):
    backups = ".backup"
    mkdir_p(path.join(calib_path, backups))

    backup = path.join(backups, 
      find_unique(path.join(calib_path, backups), filename))

    rospy.loginfo(f"Backing up calibration {filename} to {backup}")
    shutil.copy(calibration_filename, path.join(calib_path, backup))

  rospy.loginfo(f"Writing calibration to {filename}")
  with open(calibration_filename, "wt") as file:
    file.write(calibration)

def exceptions_to_rosout():
  """Calls logfatal before raising base exception"""
  def rosout_except(err_type, value, tb):
    traceback_formatted = '\n'.join(traceback.extract_tb(tb).format())
    rospy.logfatal(
        f'\n{traceback_formatted} {value.__class__.__name__}: {value}'
    )
    time.sleep(1) # Gives time to send message
    sys.__excepthook__(err_type, value, tb)
  sys.excepthook = rosout_except