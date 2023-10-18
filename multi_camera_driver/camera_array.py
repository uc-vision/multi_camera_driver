#!/usr/bin/env python3

"""ROS node for publishing images from sets of Spinnaker SDK based cameras.

Copyright (c) 2019. UC Vision Group, Canterbury University. This file is subject to the 3-clause BSD
license, as found in the LICENSE file in the top-level directory of this
distribution and at https://github.com/sds53/spinnaker_camera_driver_ros/LICENSE.
No part of multi_camera_driver.helpers, including this file, may be copied, modified,
propagated, or distributed except according to the terms contained in the
LICENSE file.
"""

from dataclasses import replace
from functools import partial
from pathlib import Path
from typing import List
import PySpin
from beartype import beartype
import rospy2 as rospy

import torch

from multi_camera_driver.helpers.camera_node import CameraArrayNode
from multi_camera_driver.helpers.image_processor.outputs import ImageOutputs
from multi_camera_driver.helpers.publisher import FramePublisher
import tf2_ros
from std_msgs.msg import String
import numpy as np

from multi_camera_driver.helpers.camera_set import CameraSet
from multi_camera_driver.helpers import spinnaker_helpers
from multi_camera_driver.helpers.camera_params import declare_ros2_parameters
#from multi_camera_driver.helpers.diagnostics import CameraDiagnosticUpdater
from multi_camera_driver.helpers.image_processor.frame_processor import FrameProcessor
from multi_camera_driver.helpers.sync_handler import SyncHandler

from multi_camera_driver.helpers.image_settings import ImageSettings
from multi_camera_driver.helpers.config import (import_calibrations, 
  load_calibrations, load_config, publish_extrinsics, write_calibration, exceptions_to_rosout)


from traceback import format_exc
import gc

def init_calibrations(camera_ids):
  calibration_file = rospy.get_param("calibration_file", '')
  tracking_frame = rospy.get_param("tracking_frame", '')

  if tracking_frame == '':
    tracking_frame = None
  if calibration_file == '':
    calibration_file = None

  calib = load_calibrations(calibration_file, camera_ids, tracking_frame)
  broadcaster = tf2_ros.StaticTransformBroadcaster()
  publish_extrinsics(broadcaster, calib, camera_ids)

  def on_recalibrated(msg):
    rospy.loginfo("Recieved recalibration, importing")
    try:
      calib = import_calibrations(msg.data, camera_ids, tracking_frame)

      # image_publisher.update_calibration(calib.cameras)
      publish_extrinsics(broadcaster, calib, camera_ids)
      write_calibration(calibration_file, msg.data)
    except:
      rospy.logerr(f"on_recalibrated exception: {format_exc()}")

  recalibrate_sub = rospy.Subscriber("recalibrated", String, on_recalibrated)
  return calib, recalibrate_sub


class ImageWriterRaw:
  def __init__(self, base_path:Path, camera_ids:List[str]):
    self.base_path = Path(base_path)

    self.camera_paths = [self.base_path / f"{id}" for id in camera_ids]
    for path in self.camera_paths:
      path.mkdir(parents=True, exist_ok=True)

    self.camera_ids = camera_ids

  def write(self, outputs:List[ImageOutputs]):

    for i, output in enumerate(outputs):
      raw = output.raw
      path = self.camera_paths[i] / f"{raw.seq}.npy"
      np.save(path, raw.image_data, allow_pickle=True)


def run_node(camera_set_file, camera_settings_file):
  camera_set_dict = load_config(camera_set_file)
  camera_settings_dict = load_config(camera_settings_file)

  camera_set = CameraSet(
      camera_serials=camera_set_dict.get("cameras"),
      camera_settings = camera_settings_dict.get("camera_settings"),
      interface_settings = camera_set_dict.get("interface_settings", []),
      master_id=camera_set_dict.get("master", None),
      trigger_reporter=camera_set_dict.get("trigger_reporter", None)
  )

  calib, recalibrated = init_calibrations(camera_set.camera_ids)
  camera_set.update_calibration(calib.cameras)

  camera_node = CameraArrayNode(camera_set)

  handler = SyncHandler(camera_set.camera_settings, 
                        timeout_msec=rospy.get_param("timeout_msec", 1000), 
                        sync_threshold_msec=rospy.get_param("sync_threshold_msec", 10),
                        device=torch.device(camera_node.image_settings.device))
  camera_set.bind(on_image=handler.publish)
  camera_set.bind(on_trigger_time=handler.trigger_time)

  #diagnostics = CameraDiagnosticUpdater(camera_set.camera_settings, camera_set.master_id)
  #camera_set.bind(on_settings=diagnostics.on_camera_info, on_image=diagnostics.on_image)

  #camera_node.bind(on_update=diagnostics.reset)
  camera_node.bind(on_update=handler.report_recieved)

  processor = FrameProcessor(camera_set.camera_settings, camera_node.image_settings)
  handler.bind(on_frame=processor.process)
  handler.bind(on_dsync=camera_node.resync)
  camera_node.bind(on_image_settings=processor.update_settings)

  publisher = FramePublisher(camera_set.camera_ids)
  processor.bind(on_frame=publisher.publish)

  # raw_writer = ImageWriterRaw("/home/oliver/raw_images", camera_set.camera_ids)
  # processor.bind(on_frame=raw_writer.write)

  
  declare_ros2_parameters(camera_settings_dict)

  try:
    camera_node.capture()
  except KeyboardInterrupt:
    pass
  except PySpin.SpinnakerException as e:
    rospy.logerr("Error capturing:" + str(e))

  recalibrated.unregister()

  handler.stop()
  processor.stop()
  publisher.stop()
  camera_node.stop()
  camera_node.cleanup()

  del camera_node
  del camera_set
  del handler
  del processor
  del publisher
  #del diagnostics

def main():
  rospy.init_node('camera_array', anonymous=False)
  camera_set_file = rospy.get_param("camera_set_file", '')
  camera_settings_file = rospy.get_param("settings_file", '')

  rospy.loginfo(str(rospy.get_param("reset_cycle", True)))
  if rospy.get_param("reset_cycle", True):
    spinnaker_helpers.reset_all()
    rospy.sleep(1)

  rospy.loginfo("Starting")

  system = PySpin.System.GetInstance()

  with torch.inference_mode():
    run_node(camera_set_file, camera_settings_file)
  
  gc.collect(generation=0)
  rospy.sleep(2.0)
  gc.collect(generation=0)

  system.ReleaseInstance()


if __name__ == "__main__":
  main()
