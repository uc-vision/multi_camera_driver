#!/usr/bin/env python
"""ROS node for spinnaker cameras.

Copyright (c) 2019. Sam Schofield. This file is subject to the 3-clause BSD
license, as found in the LICENSE file in the top-level directory of this
distribution and at https://github.com/sds53/spinnaker_camera_driver_ros/LICENSE.
No part of spinnaker_camera_driver_helpers, including this file, may be copied, modified,
propagated, or distributed except according to the terms contained in the
LICENSE file.
"""

from __future__ import print_function

import cv2
import threading
import yaml

import PySpin
import rospy

from std_srvs.srv import Empty, EmptyResponse
from camera_geometry_ros.conversions import camera_info_msg

from sensor_msgs.msg import CameraInfo

from spinnaker_camera_driver_helpers import spinnaker_helpers
from spinnaker_camera_driver_helpers.common import *
from spinnaker_camera_driver_ros.cfg import CameraArraySettingsConfig

from dynamic_reconfigure.server import Server

LOCK = threading.Lock()

class ImageEventHandler(PySpin.ImageEvent):
    def __init__(self, publisher, camera):
        super(ImageEventHandler, self).__init__()
        self.publisher = publisher

        self.sent = False
        self.frame = 0
        self.stamp = rospy.Time.now()
        self.node_map = camera.GetNodeMap()

    def OnImageEvent(self, image):
        if image.IsIncomplete():
            print('Image incomplete with image status %d ...' % image.GetImageStatus())
        else:
            image_data = image.GetNDArray()
            image.Release()

            self.publisher.publish(image_data, self.stamp, encoding="bayer_rggb8")
            self.sent = True


def init_camera(camera, image_topic, camera_info=CameraInfo(), trigger_master=None, desc='', camera_settings=None):
    camera.Init()

    publisher = ImagePublisher(image_topic, camera_info=camera_info)
    event_handler = ImageEventHandler(publisher, camera)

    nodemap_tldevice = camera.GetTLDeviceNodeMap()
    serial = PySpin.CStringPtr(nodemap_tldevice.GetNode('DeviceSerialNumber'))
    print("Initialising: ", desc, serial.GetValue())
    spinnaker_helpers.set_camera_settings(camera, camera_settings)

    if trigger_master is not None:
        spinnaker_helpers.enable_triggering(camera, trigger_master)

    camera.RegisterEvent(event_handler)
    camera.BeginAcquisition()
    return event_handler


def set_exposure(camera, exposure_time):
    """Set the cameras exposure time the the given value. If 0 set to auto"""
    node_map = camera.GetNodeMap()
    if exposure_time > 0:
        spinnaker_helpers.set_enum(node_map, "ExposureAuto", "Off")
        spinnaker_helpers.set_float(node_map, "ExposureTime", exposure_time)
    else:
        spinnaker_helpers.set_enum(node_map, "ExposureAuto", "Continuous")


def set_balance_ratio(camera, balance_ratio):
    node_map = camera.GetNodeMap()
    spinnaker_helpers.set_float(node_map, "BalanceRatio", balance_ratio)


class CameraArrayNode(object):
    def __init__(self, config=None, calibrations={}):
        self.system = PySpin.System.GetInstance()
        self.output_dir = config.get("output_dir", "")
        self.camera_settings = config.get("camera_settings", None)
        self.calibrations = calibrations

        self.master_id = config.get("master", None)
        self.camera_serials = config.get("camera_aliases", None)  # serial -> alias
        self.camera_aliases = {v: k for k, v in self.camera_serials.iteritems()}  # alias -> serial
        self.event_handlers = []
        self.cameras_initialised = False

        camera_list = self.system.GetCameras()
        self.camera_dict = spinnaker_helpers.camera_list_to_dict(camera_list)  # serial -> camera
        self.warn_cameras_missing()

        self.reconfigure_srv = Server(CameraArraySettingsConfig, self.reconfigure_callback)
        self.timeout = config.get("timeout", 1.0)

    def get_cam_by_alias(self, alias):
        return self.camera_dict[self.camera_aliases[alias]]

    def reconfigure_callback(self, config, _):

        if self.cameras_initialised:
            top_exposure = config.get("top_exposure_time")
            mid_exposure = config.get("mid_exposure_time")
            bottom_exposure = config.get("bottom_exposure_time")

            if top_exposure is not None:
                set_exposure(self.get_cam_by_alias("cam1"), top_exposure)
                set_exposure(self.get_cam_by_alias("cam2"), top_exposure)

            if mid_exposure is not None:
                set_exposure(self.get_cam_by_alias("cam3"), mid_exposure)
                set_exposure(self.get_cam_by_alias("cam4"), mid_exposure)

            if bottom_exposure is not None:
                set_exposure(self.get_cam_by_alias("cam5"), bottom_exposure)
                set_exposure(self.get_cam_by_alias("cam6"), bottom_exposure)

            if 'balance_ratio' in config and config["balance_ratio"] > 0:
                balance_ratio = config["balance_ratio"]
                for camera in self.camera_dict.values():
                    set_balance_ratio(camera, balance_ratio)
        return config


    def warn_cameras_missing(self):
        if self.camera_serials is not None:
            for serial in self.camera_serials.keys():
                if serial not in self.camera_dict.keys():
                    print("Camera not found: " + str(serial))

    def trigger(self):
        spinnaker_helpers.trigger(self.camera_dict.get(self.master_id, self.camera_dict.values()[0]))

    def start(self):
        print("Starting camera")
        if len(self.camera_dict) == 0:
            print("No cameras found")
            return

        event_handlers = []
        started = []
        for serial, camera in self.camera_dict.items():
            if self.camera_serials is None:
                alias = "cam_{}".format(serial)
            else:
                alias = self.camera_serials.get(serial, "cam_{}".format(serial))

            cam_info = CameraInfo()
            if alias in self.calibrations:
                cam_info = camera_info_msg(self.calibrations[alias])

            event_handler = init_camera(camera, alias, cam_info, serial == self.master_id, alias, self.camera_settings)
            event_handlers.append(event_handler)
            started.append(camera)
            del camera

        self.cameras_initialised = True

        if len(event_handlers) > 0:
            print("Acquisition Started")

            stamp = rospy.Time.now()
            for handler in event_handlers:
                handler.stamp = stamp

            self.trigger()

            while not rospy.is_shutdown():
                ready = all([handler.sent for handler in event_handlers])
                delay = rospy.Time.now() - stamp

                if delay.to_sec() > self.timeout:
                    print("Capture timed out, re-triggering")
                    stamp = rospy.Time.now()
                    self.trigger()

                if ready:
                    stamp = rospy.Time.now()
                    with LOCK:
                        for handler in event_handlers:
                            handler.sent = False
                            handler.stamp = stamp
                        self.trigger()

            print("Acquisition ended")
            for camera in started:
                camera.EndAcquisition()

                # TODO: UnregisterEventHandler?

    def stop(self):
        print("Stopping camera")
        for camera in self.camera_dict.values():
            spinnaker_helpers.load_defaults(camera)
            # Why are we doing this here?
            # Otherwise the camera can't be examined in spinview (trigger mode etc.)
            camera.DeInit()
        del camera
        del self.camera_dict
        self.system.ReleaseInstance()



def main():
    print("Starting")
    spinnaker_helpers.reset_all()

    rospy.init_node('camera_array_node', anonymous=False)
    config_file = rospy.get_param("~config_file", None)
    calibration_file = rospy.get_param("~calibration_file", None)

    config = load_config(config_file)  
    camera_calibs, extrinsics = load_calibration(calibration_file)

    broadcaster = publish_extrinsics(extrinsics)
    camera_node = CameraArrayNode(config, camera_calibs)

    stereo_pairs = config.get('stereo_pairs') or {}
    stereo_processors = [StereoPublisher(name, left, right) 
        for name, (left, right) in stereo_pairs.items()]

    try:
        camera_node.start()
    except KeyboardInterrupt:
        pass
    camera_node.stop()


if __name__ == "__main__":
    main()
