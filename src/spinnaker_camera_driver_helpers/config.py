import numpy as np
import rospy

import tf2_ros
import yaml

from functools import partial

import camera_geometry_ros.conversions as conversions
from camera_geometry.calib import import_rig
from camera_geometry import json


def load_config(config_file):
    if config_file is not None:
        with open(config_file) as config_file:
            return yaml.load(config_file, Loader=yaml.Loader)
    else:
        return None

def import_calibrations(calib_string, camera_names):
  calib = json.load_string(calib_string)
  camera_calibrations = import_rig(calib)

  if set(camera_names) != set(camera_calibrations.keys()):
    rospy.logwarn(f"calibrations {set(camera_calibrations.keys())} don't match cameras {set(camera_names)}")

  return camera_calibrations


def load_calibrations(calibration_file, camera_names):
    rospy.loginfo(f"Loading calibrations from: {calibration_file}")

    camera_calibrations = {}
    try:
      if calibration_file is not None:
          calib = json.load_json(calibration_file)
          camera_calibrations = import_rig(calib)

      if set(camera_names) != set(camera_calibrations.keys()):
        rospy.logwarn(f"calibrations {set(camera_calibrations.keys())} don't match cameras {set(camera_names)}")

    except FileNotFoundError:
        rospy.logwarn(f"Calibration file not found: {calibration_file}")


    return camera_calibrations

def publish_transforms(broadcaster, namespace, transforms):
    stamp = rospy.Time.now()

    msgs = [conversions.transform_msg(parent_to_cam, namespace, f"{namespace}/{child_id}", stamp)
            for child_id, parent_to_cam in transforms.items()]
    broadcaster.sendTransform(msgs)
    
def publish_extrinsics(broadcaster, calibrations, camera_names):
  found = {k:camera.parent_to_camera 
    for k, camera in  calibrations.items()}

  extrinsics = {alias:found.get(alias, np.eye(4))
    for alias in camera_names}

  namespace = rospy.get_namespace().strip('/')
  publish_transforms(broadcaster, namespace, extrinsics)   