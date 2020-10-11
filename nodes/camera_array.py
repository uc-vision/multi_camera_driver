#!/usr/bin/env python3
"""ROS node for spinnaker cameras.

Copyright (c) 2019. Sam Schofield. This file is subject to the 3-clause BSD
license, as found in the LICENSE file in the top-level directory of this
distribution and at https://github.com/sds53/spinnaker_camera_driver_ros/LICENSE.
No part of spinnaker_camera_driver_helpers, including this file, may be copied, modified,
propagated, or distributed except according to the terms contained in the
LICENSE file.
"""

from __future__ import print_function

import cv_bridge
import cv2

import threading
import yaml
import math

from time import sleep

import PySpin

from std_srvs.srv import Empty, EmptyResponse

from sensor_msgs.msg import CameraInfo

from dynamic_reconfigure.server import Server
import sys
import traceback

from spinnaker_camera_driver_helpers import spinnaker_helpers
from spinnaker_camera_driver_helpers.common import *
from spinnaker_camera_driver_ros.cfg import CameraArraySettingsConfig
import tf2_ros



ImageEvent = getattr(PySpin, 'ImageEventHandler', None) or getattr(PySpin, 'ImageEvent')

class ImageEventHandler(ImageEvent):
    def __init__(self, publisher, camera):
        super(ImageEventHandler, self).__init__()
        self.publisher = publisher

        self.sent = False
        self.frame = 0
        self.stamp = rospy.Time.now()
        self.node_map = camera.GetNodeMap()

    def OnImageEvent(self, image):
        try:
            self.publisher.publish(image, self.stamp)
            self.sent = True
        except:
            traceback.print_exc(file=sys.stdout)
            sys.exit(1)

    def stop(self):
        self.publisher.stop()

class SpinnakerPublisher(object):
    def __init__(self, publisher):
        self.publisher = publisher

    def publish(self, image, stamp):
        if image.IsIncomplete():
            rospy.logerr('Image incomplete with image status %d ...' % image.GetImageStatus())
        else:
            image_data = image.GetNDArray()
            image.Release()

            self.publisher.publish(image_data, stamp)

    def stop(self):
        self.publisher.stop()

def init_camera(camera, image_topic, calibration=None, trigger_master=None, desc='', camera_settings=None, encoding="bayer_rggb8"):
    try:
        camera.Init()

        publisher = CalibratedPublisher(image_topic, calibration=calibration, encoding=encoding)
        publisher = SpinnakerPublisher(publisher)
        
        event_handler = ImageEventHandler(AsyncPublisher(publisher), camera)

        nodemap_tldevice = camera.GetTLDeviceNodeMap()
        serial = PySpin.CStringPtr(nodemap_tldevice.GetNode('DeviceSerialNumber'))
        rospy.loginfo("Initialising: {} {}".format(desc, serial.GetValue()))

        spinnaker_helpers.set_camera_settings(camera, camera_settings)

        if trigger_master is not None:
            spinnaker_helpers.enable_triggering(camera, trigger_master)

        if getattr(camera, 'RegisterEvent', None):
            camera.RegisterEvent(event_handler)
        else:
            camera.RegisterEventHandler(event_handler)
            
        camera.BeginAcquisition()
        return event_handler
    except PySpin.SpinnakerException as e:
        publisher.stop()
        rospy.logerr("Could not initialise camera {}: {}".format(desc, str(e)))


def set_exposure(camera, exposure_time):
    """Set the cameras exposure time the the given value. If 0 set to auto"""
    node_map = camera.GetNodeMap()
    if exposure_time > 0:
        spinnaker_helpers.set_enum(node_map, "ExposureAuto", "Off")
        spinnaker_helpers.set_float(node_map, "ExposureTime", exposure_time)
    else:
        spinnaker_helpers.set_enum(node_map, "ExposureAuto", "Continuous")


def set_gain(camera, gain):
    """Set the cameras exposure time the the given value. If 0 set to auto"""
    node_map = camera.GetNodeMap()
    if gain > 0:
        spinnaker_helpers.set_enum(node_map, "GainAuto", "Off")
        spinnaker_helpers.set_float(node_map, "Gain", gain)
    else:
        spinnaker_helpers.set_enum(node_map, "GainAuto", "Continuous")


def set_balance_ratio(camera, balance_ratio):
    node_map = camera.GetNodeMap()
    spinnaker_helpers.set_float(node_map, "BalanceRatio", balance_ratio)


class CameraArrayNode(object):
    def __init__(self, config=None, calibrations={}):
        self.system = PySpin.System.GetInstance()

        self.output_dir = config.get("output_dir", "")
        self.camera_settings = config.get("camera_settings", None)
        self.encoding = config.get("encoding", "bayer_rggb8")

        self.calibrations = calibrations

        self.master_id = config.get("master", None)
        self.camera_serials = config.get("camera_aliases", None)  # serial -> alias
        self.camera_aliases = {v: k for k, v in self.camera_serials.items()}  # alias -> serial
        self.event_handlers = []
        self.cameras_initialised = False

        camera_list = self.system.GetCameras()
        self.camera_dict = spinnaker_helpers.camera_list_to_dict(camera_list)  # serial -> camera
        self.warn_cameras_missing()

        self.reconfigure_srv = Server(CameraArraySettingsConfig, self.reconfigure_callback)

        self.timeout = config.get("timeout", 1.0)
        self.max_rate = config.get("max_framerate", math.inf)


    def get_cam_by_alias(self, alias):
        return self.camera_dict[self.camera_aliases[alias]]

    def reconfigure_callback(self, config, _):

        if self.cameras_initialised:
            if 'exposure' in config and config["exposure"] > 0:
                balance_ratio = config["exposure"]
                for camera in self.camera_dict.values():
                    set_exposure(camera, balance_ratio)

            if 'balance_ratio' in config and config["balance_ratio"] > 0:
                balance_ratio = config["balance_ratio"]
                for camera in self.camera_dict.values():
                    set_balance_ratio(camera, balance_ratio)

            if "gain" in config and config["gain"] > 0:
                gain = config["gain"]
                for camera in self.camera_dict.values():
                    set_gain(camera, gain)
        return config


    def warn_cameras_missing(self):
        if self.camera_serials is not None:
            for serial in self.camera_serials.keys():
                if serial not in self.camera_dict.keys():
                    rospy.logerr("Camera not found: " + str(serial))

        assert self.master_id in self.camera_dict, "master camera {} not found".format(self.master_id)

    def trigger(self):
        assert self.master_id in self.camera_dict
        try:
            spinnaker_helpers.trigger(self.camera_dict[self.master_id])
        except PySpin.SpinnakerException as e:
            rospy.logerr("Error triggering: ", e)


    def start(self):
        rospy.loginfo("Starting cameras")
        if len(self.camera_dict) == 0:
            rospy.logerr("No cameras found")
            return

        event_handlers = []
        started = []
        for serial, camera in self.camera_dict.items():
            if self.camera_serials is None:
                alias = "cam_{}".format(serial)
            else:
                alias = self.camera_serials.get(serial, "cam_{}".format(serial))

            event_handler = init_camera(camera, alias, self.calibrations.get(alias, None), 
                serial == self.master_id, alias, self.camera_settings, self.encoding)

            if event_handler is not None:
                event_handlers.append(event_handler)
                started.append(camera)
            del camera

        self.cameras_initialised = True

        if len(event_handlers) > 0:
            rospy.loginfo("Acquisition Started")

            stamp = rospy.Time.now()
            for handler in event_handlers:
                handler.stamp = stamp

            self.trigger()

            while not rospy.is_shutdown():
                ready = all([handler.sent for handler in event_handlers])
                delay = rospy.Time.now() - stamp

                if delay.to_sec() > self.timeout:
                    rospy.logwarn("Capture timed out, re-triggering")
                    stamp = rospy.Time.now()
                    self.trigger()

                if ready and delay.to_sec() > 1.0/self.max_rate:
                    stamp = rospy.Time.now()
                    for handler in event_handlers:
                        handler.sent = False
                        handler.stamp = stamp
                    self.trigger()

            rospy.loginfo("Acquisition ended")
            for camera in started:
                camera.EndAcquisition()

                # TODO: UnregisterEventHandler?
            for handler in event_handlers:
                handler.stop()

    def stop(self):

        rospy.loginfo("Stopping cameras")
        for camera in self.camera_dict.values():
            spinnaker_helpers.load_defaults(camera)
            # Why are we doing this here?
            # Otherwise the camera can't be examined in spinview (trigger mode etc.)
            camera.DeInit()
        del camera
        del self.camera_dict
        self.system.ReleaseInstance()



def main():
    rospy.init_node('camera_array_node', anonymous=False)

    rospy.loginfo("Starting")
    spinnaker_helpers.reset_all()

    config_file = rospy.get_param("~config_file", None)
    calibration_file = rospy.get_param("~calibration_file", None)

    config = load_config(config_file)
    camera_calibs, extrinsics, _ = load_calibration(calibration_file)
    broadcaster = publish_extrinsics(extrinsics)
    camera_node = CameraArrayNode(config, camera_calibs)

    # stereo_pairs = config.get('stereo_pairs') or {}
    # stereo_processors = [StereoPublisher(name, left, right)
    #     for name, (left, right) in stereo_pairs.items()]

    try:
        camera_node.start()
    except KeyboardInterrupt:
        pass
    camera_node.stop()


if __name__ == "__main__":
    main()
