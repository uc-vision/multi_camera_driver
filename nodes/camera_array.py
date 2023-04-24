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
from functools import partial
from pathlib import Path
from typing import List
import PySpin
from beartype import beartype

import rospy
import rospkg

from spinnaker_camera_driver_helpers.camera_node import CameraArrayNode
from spinnaker_camera_driver_helpers.image_processor.outputs import ImageOutputs
from spinnaker_camera_driver_helpers.publisher import FramePublisher
import tf2_ros
from std_msgs.msg import String
import numpy as np

from spinnaker_camera_driver_helpers.camera_set import CameraSet
from spinnaker_camera_driver_helpers import spinnaker_helpers
from spinnaker_camera_driver_helpers.diagnostics import CameraDiagnosticUpdater
from spinnaker_camera_driver_helpers.image_processor.frame_processor import FrameProcessor
from spinnaker_camera_driver_helpers.sync_handler import SyncHandler


from spinnaker_camera_driver_helpers.image_settings import ImageSettings
from spinnaker_camera_driver_helpers.config import (import_calibrations, 
  load_calibrations, load_config, publish_extrinsics, write_calibration, exceptions_to_rosout)


from traceback import format_exc
import gc




def init_calibrations(camera_ids):
  calibration_file = rospy.get_param("~calibration_file", None)
  tracking_frame = rospy.get_param("tracking_frame", None)

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


def base_settings():
    return ImageSettings(
      device=rospy.get_param("~device", 'cuda:0'),
      jpeg_quality=95,
      preview_size=rospy.get_param("~preview_width", 200),
      resize_width=rospy.get_param("~resize_width", 0),
  )


def run_node():

  camera_set = CameraSet(
      camera_serials=rospy.get_param("~cameras"),
      camera_settings = rospy.get_param("~camera_settings"),
      master_id=rospy.get_param("~master")
  )

  calib, recalibrated = init_calibrations(camera_set.camera_ids)
  camera_set.update_calibration(calib.cameras)

  camera_node = CameraArrayNode(camera_set, base_settings())

  handler = SyncHandler(camera_set.camera_settings)
  camera_set.bind(on_image=handler.publish)

  diagnostics = CameraDiagnosticUpdater(camera_set.camera_settings, camera_set.master_id)
  camera_set.bind(on_settings=diagnostics.on_camera_info, on_image=diagnostics.on_image)

  camera_node.bind(on_update=diagnostics.reset)
  camera_node.bind(on_update=handler.report_recieved)

  processor = FrameProcessor(camera_set.camera_settings, camera_node.image_settings)
  handler.bind(on_frame=processor.process)
  camera_node.bind(on_image_settings=processor.update_settings)

  publisher = FramePublisher(camera_set.camera_ids)
  processor.bind(on_frame=publisher.publish)

  # raw_writer = ImageWriterRaw("/home/oliver/raw_images", camera_set.camera_ids)
  # processor.bind(on_frame=raw_writer.write)


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
  del diagnostics

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
