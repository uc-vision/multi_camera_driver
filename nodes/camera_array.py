#!/usr/bin/env python3

"""ROS node for publishing images from sets of Spinnaker SDK based cameras.

Copyright (c) 2019. UC Vision Group, Canterbury University. This file is subject to the 3-clause BSD
license, as found in the LICENSE file in the top-level directory of this
distribution and at https://github.com/sds53/spinnaker_camera_driver_ros/LICENSE.
No part of spinnaker_camera_driver_helpers, including this file, may be copied, modified,
propagated, or distributed except according to the terms contained in the
LICENSE file.
"""

from dataclasses import replace
from pathlib import Path
import PySpin

import rospy
import rospkg

from spinnaker_camera_driver_helpers.camera_node import CameraArrayNode
import tf2_ros
from std_msgs.msg import String


from spinnaker_camera_driver_helpers.camera_set import CameraSet
from spinnaker_camera_driver_helpers import spinnaker_helpers
from spinnaker_camera_driver_helpers.image_handler import ImageHandler
from spinnaker_camera_driver_helpers.sync_handler import SyncHandler


from spinnaker_camera_driver_helpers.image_settings import ImageSettings
from spinnaker_camera_driver_helpers.config import (import_calibrations, 
  load_calibrations, load_config, publish_extrinsics, write_calibration, exceptions_to_rosout)


from traceback import format_exc
import gc


def base_settings(config):
  default_backend = "turbo_jpeg"
  try:
    import nvjpeg_torch
    import torch

    if torch.cuda.is_available():
      default_backend = "torch_nvjpeg"
  except ModuleNotFoundError as e:
    rospy.logwarn(f"Could not import nvjpeg_torch: {e}")
    pass

  rospack = rospkg.RosPack()
  base_path = Path(rospack.get_path('spinnaker_camera_driver_ros'))


  return ImageSettings(
      device=rospy.get_param("~device", 'cuda:0'),
      jpeg_quality=95,
      preview_size=rospy.get_param("~preview_width", 200),
      image_backend=rospy.get_param("~backend", default_backend),
      cache_path = rospy.get_param("~cache_path", base_path / "cache"),
      resize_width=rospy.get_param("~resize_width", 0),
      sharpen=rospy.get_param("~sharpen", 0.0)
  )


def create_publisher(camera_set:CameraSet, settings:ImageSettings):
  handler_type = ImageHandler if camera_set.master_id is None else SyncHandler
  return handler_type(settings, camera_set)


def run_node():
  config_file = rospy.get_param("~config_file")
  config = load_config(config_file)

  camera_set = CameraSet(
      camera_serials=config["camera_aliases"],
      camera_settings=config["camera_settings"],
      master_id=config.get("master", None),
  )

  calibration_file = rospy.get_param("~calibration_file", None)
  tracking_frame = rospy.get_param("tracking_frame", None)

  calib = load_calibrations(calibration_file, camera_set.camera_ids, tracking_frame)
  broadcaster = tf2_ros.StaticTransformBroadcaster()
  publish_extrinsics(broadcaster, calib, camera_set.camera_ids)

  
  image_publisher = create_publisher(camera_set, base_settings(config))

  def on_recalibrated(msg):
    rospy.loginfo("Recieved recalibration, importing")
    try:
      calib = import_calibrations(msg.data, camera_set.camera_ids, tracking_frame)

      image_publisher.update_calibration(calib.cameras)
      publish_extrinsics(broadcaster, calib, camera_set.camera_ids)

      write_calibration(calibration_file, msg.data)
    except:
      rospy.logerr(f"on_recalibrated exception: {format_exc()}")

  recalibrate_sub = rospy.Subscriber("recalibrated", String, on_recalibrated)

  camera_node = CameraArrayNode(image_publisher, camera_set)

  try:
    camera_node.capture()
  except KeyboardInterrupt:
    pass
  except PySpin.SpinnakerException as e:
    rospy.logerr("Error capturing:" + str(e))

  recalibrate_sub.unregister()

  image_publisher.stop()
  camera_node.stop()
  camera_node.cleanup()

  del camera_node
  del camera_set
  del image_publisher

def main():
  exceptions_to_rosout()
  rospy.init_node('camera_array_node', anonymous=False)

  if rospy.get_param("~reset_cycle", True):
    spinnaker_helpers.reset_all()
    rospy.sleep(1)

  rospy.loginfo("Starting")

  system = PySpin.System.GetInstance()

  run_node()
  
  gc.collect(generation=0)
  rospy.sleep(2.0)
  gc.collect(generation=0)

  system.ReleaseInstance()


if __name__ == "__main__":
  main()
