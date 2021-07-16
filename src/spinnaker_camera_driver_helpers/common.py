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


class Lazy(object):
    def __init__(self, f, *args, **kwargs):
        self.f = partial(f, *args, **kwargs)
        self.result = None

    def get(self):
        self.result = self.f() if self.result is None else self.result
        return self.result



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

def publish_extrinsics(namespace, transforms):

    stamp = rospy.Time.now()
    broadcaster = tf2_ros.StaticTransformBroadcaster()

    msgs = [conversions.transform_msg(parent_to_cam, namespace, f"{namespace}/{child_id}", stamp)
            for child_id, parent_to_cam in transforms.items()]

    broadcaster.sendTransform(msgs)
    return broadcaster
