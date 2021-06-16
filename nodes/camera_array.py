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

from camera_geometry.json import load_json

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

import gc


ImageEvent = getattr(PySpin, 'ImageEventHandler', None) or getattr(PySpin, 'ImageEvent')

class ImageEventHandler(ImageEvent):
    def __init__(self, publisher, camera):
        super(ImageEventHandler, self).__init__()
        self.publisher = publisher

        self.sent = False
        self.frame = 0
        self.stamp = None
        self.node_map = camera.GetNodeMap()

    def OnImageEvent(self, image):
        try:
            if self.stamp is not None:
                self.publisher.publish(image, self.stamp)
                self.sent = True
        except:
            traceback.print_exc(file=sys.stdout)
            sys.exit(1)

    def stop(self):
        self.stamp = None
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




def set_exposure(camera, exposure_time):
    """Set the cameras exposure time the the given value. If 0 set to auto"""
    node_map = camera.GetNodeMap()
    if exposure_time > 0:
        spinnaker_helpers.set_enum(node_map, "ExposureAuto", "Off")
        return spinnaker_helpers.try_set_float(node_map, "ExposureTime", exposure_time)
    else:
        spinnaker_helpers.set_enum(node_map, "ExposureAuto", "Continuous")


def set_gain(camera, gain):
    """Set the cameras exposure time the the given value. If 0 set to auto"""
    node_map = camera.GetNodeMap()
    if gain > 0:
        spinnaker_helpers.set_enum(node_map, "GainAuto", "Off")
        return spinnaker_helpers.try_set_float(node_map, "Gain", gain)
    else:
        spinnaker_helpers.set_enum(node_map, "GainAuto", "Continuous")


def set_balance_ratio(camera, balance_ratio):
    node_map = camera.GetNodeMap()
    if balance_ratio >= 0.5:
        spinnaker_helpers.set_enum(node_map, "BalanceWhiteAuto", "Off")
        return spinnaker_helpers.try_set_float(node_map, "BalanceRatio", balance_ratio)
    else:
        spinnaker_helpers.set_enum(node_map, "BalanceWhiteAuto", "Continuous")

def set_grey_value(camera, value):
    """Set the target grey value. If 0 set to auto"""
    node_map = camera.GetNodeMap()
    if value > 0:
        spinnaker_helpers.set_enum(node_map, "AutoExposureTargetGreyValueAuto", "Off")
        return spinnaker_helpers.try_set_float(node_map, "AutoExposureTargetGreyValue", value)
    else:
        spinnaker_helpers.set_enum(node_map, "AutoExposureTargetGreyValueAuto", "Continuous")


def set_ev_comp(camera, value):
    """Set the EV Compensation"""
    node_map = camera.GetNodeMap()
    return spinnaker_helpers.try_set_float(node_map, "AutoExposureEVCompensation", value)


def set_gamma(camera, value):
    """Set the gamma value. If 0 set to off"""
    node_map = camera.GetNodeMap()
    if value > 0:
        spinnaker_helpers.set_bool(node_map, "GammaEnable", True)
        return spinnaker_helpers.try_set_float(node_map, "Gamma", value)
    else:
        spinnaker_helpers.set_bool(node_map, "GammaEnable", False)
        

def set_binning(camera, value):
    """Set the gamma value. If 0 set to off"""
    node_map = camera.GetNodeMap()
    spinnaker_helpers.try_set_int(node_map, "BinningHorizontal", value)
    spinnaker_helpers.try_set_int(node_map, "BinningVertical", value)

    w = spinnaker_helpers.get_int(node_map, "WidthMax")
    h = spinnaker_helpers.get_int(node_map, "HeightMax")

    spinnaker_helpers.try_set_int(node_map, "Width", w)
    spinnaker_helpers.try_set_int(node_map, "Height", h)




def set_internal(camera, value):
    """Set internal setting (no camera setting)"""
    pass

property_setters = dict(
    exposure = set_exposure,
    balance_ratio = set_balance_ratio,
    gain = set_gain,
    grey_value = set_grey_value,
    ev_comp = set_ev_comp,
    gamma = set_gamma,
    max_framerate = set_internal
)

delayed_setters = dict(    
    binning = set_binning
)

class CameraArrayNode(object):
    def __init__(self, config, calibrations={}, preview_sizes={}):
        self.system = PySpin.System.GetInstance()
        assert config is not None

        self.output_dir = config.get("output_dir", "")
        self.camera_settings = config.get("camera_settings", None)
        self.raw_encoding = config.get("encoding", "bayer_rggb8")

        self.camera_serials = config.get("camera_aliases", None) # serial -> alias
        self.master_id = config.get("master", None)   

        self.free_running = config.get("free_running", False)
        self.camera_dict = spinnaker_helpers.find_cameras(self.camera_serials)  # alias -> camera

        assert self.master_id is None or self.master_id in self.camera_serials

        self.calibrations = calibrations
        self.config = {}
        self.preview_sizes = preview_sizes

        self.pending_config = {}
        
        self.event_handlers = []
        self.initialised = []
        self.cameras_initialised = False
        self.started = False

        self.reconfigure_srv = Server(CameraArraySettingsConfig, self.reconfigure_callback)
        self.timeout = config.get("timeout", 1.0)
      

    def get_cam_by_alias(self, alias):
        return self.camera_dict[alias]

    @property
    def master_name(self):
      return self.camera_serials.get(self.master_id, None)

    @property
    def master(self):
      if self.master_id is not None:
          return self.camera_dict[self.master_name]      

    def set_property(self, key, value, setter):
        try:
            rospy.loginfo(f"set_property {key}: {value}")
            for camera in self.camera_dict.values():
                setter(camera, value)      
        except PySpin.SpinnakerException as e:
            rospy.loginfo(f"set_property: {key} {value} {e} ")



    def set_config_properties(self, config):
        for k, v in config.items():
            setter = property_setters.get(k, None) or delayed_setters.get(k, None)
            if setter is None:
                return

            if self.config.get(k, None) != v and setter is not None:
                if self.started and k in delayed_setters:
                    self.pending_config[k] = v   
                else:
                    self.set_property(k, v, setter)
                    self.config[k] = v


      
    def reconfigure_callback(self, config, _):
        if self.cameras_initialised:
            self.set_config_properties(config)  
        else:
            self.pending_config.update(config)  

        return config



    def trigger(self):
        assert self.master is not None
        try:
            spinnaker_helpers.trigger(self.master)
        except PySpin.SpinnakerException as e:
            rospy.logerr("Error triggering: " + str(e))


    def start(self):
        rospy.loginfo("Begin acquisition")
        for camera in self.initialised:
           camera.BeginAcquisition()
        self.started = True

    def stop(self):
        rospy.loginfo("End acquisition")
        for camera in self.initialised:
            camera.EndAcquisition()
        self.started = False


    def initialise(self):
        rospy.loginfo("Initialising cameras")
        if len(self.camera_dict) == 0:
            rospy.logerr("No cameras found")
            return

        for alias, camera in self.camera_dict.items():

            if alias not in self.calibrations:
              rospy.logwarn(f"camera {alias} does not exist in calibrations!")

            event_handler = self.init_camera(camera, alias)

            if event_handler is not None:
                self.event_handlers.append(event_handler)
                self.initialised.append(camera)
            del camera

        self.cameras_initialised = True

        rospy.loginfo(f"Applying initial settings")
        self.update_pending()

        rospy.loginfo(f"{len(self.initialised)} Cameras initialised")

    def update_pending(self):
        if self.started and len(self.pending_config) > 0:
            rospy.loginfo(f"Applying pending settings")

            self.stop()
            self.set_config_properties(self.pending_config)
            self.start()
        else:
            self.set_config_properties(self.pending_config)

        self.pending_config = {}

    def capture(self):
      return self.capture_free() if self.free_running\
        else self.capture_sync()

    def capture_sync(self):
        assert len(self.initialised) > 0 and not self.started 
        self.start()

        stamp = rospy.Time.now()
        for handler in self.event_handlers:
            handler.stamp = stamp

        self.trigger()

        while not rospy.is_shutdown():
            ready = all([handler.sent for handler in self.event_handlers])
            delay = rospy.Time.now() - stamp

            if delay.to_sec() > self.timeout:
                rospy.logwarn("Capture timed out, re-triggering")
                stamp = rospy.Time.now()
                self.trigger()

            max_rate = self.config.get("max_framerate", math.inf)
            if ready and (max_rate == 0 or delay.to_sec() > 1.0/max_rate):
                self.update_pending()    

                stamp = rospy.Time.now()
                for handler in self.event_handlers:
                    handler.sent = False
                    handler.stamp = stamp
                self.trigger()

        self.stop()


    def capture_free(self):
        assert len(self.initialised) > 0 and not self.started 

        stamp = rospy.Time.now()
        for handler in self.event_handlers:
            handler.stamp = stamp

        self.start()

        while not rospy.is_shutdown():
            ready = all([handler.sent for handler in self.event_handlers])
            if ready:
                self.update_pending()

                for handler in self.event_handlers:
                    handler.stamp = stamp
                    handler.sent = False

        self.stop()


    def init_camera(self, camera, camera_name):
      try:
          camera.Init()

          nodemap_tldevice = camera.GetTLDeviceNodeMap()
          serial = PySpin.CStringPtr(nodemap_tldevice.GetNode('DeviceSerialNumber'))

          rospy.loginfo("Initialising: {} {}".format(camera_name, serial.GetValue()))

          spinnaker_helpers.set_camera_settings(camera, self.camera_settings)
          info = spinnaker_helpers.get_camera_info(camera)

          rospy.loginfo("{}: {}".format(camera_name, str(info)))

          if self.master_id is not None:
              spinnaker_helpers.enable_triggering(camera, master=camera_name == self.master_name, free_running=self.free_running)

          return self.publish_camera(camera_name, camera)
      except PySpin.SpinnakerException as e:
          rospy.logerr("Could not initialise camera {}: {}".format(camera_name, str(e)))


    def publish_camera(self, camera_name, camera):
      calibration = self.calibrations.get(camera_name, None)
      
      publisher = CalibratedPublisher(camera_name, calibration=calibration, 
        raw_encoding=self.raw_encoding, preview_sizes=self.preview_sizes)

      publisher = SpinnakerPublisher(publisher)
      event_handler = ImageEventHandler(AsyncPublisher(publisher), camera)

      if getattr(camera, 'RegisterEvent', None):
          camera.RegisterEvent(event_handler)
      else:
          camera.RegisterEventHandler(event_handler)
          
      return event_handler


    def cleanup(self):
        for handler in self.event_handlers:
            handler.stop()

        del self.initialised
        del self.event_handlers
        gc.collect()

        rospy.loginfo("Stopping cameras")
        for camera in self.camera_dict.values():
            spinnaker_helpers.load_defaults(camera)
            # Why are we doing this here?
            # Otherwise the camera can't be examined in spinview (trigger mode etc.)

            camera.DeInit()
        del camera
        del self.camera_dict
        gc.collect()

        self.system.ReleaseInstance()
        self.system = None
        gc.collect()

def main():
    rospy.init_node('camera_array_node', anonymous=False)

    rospy.loginfo("Starting")
    config_file = rospy.get_param("~config_file", None)
    calibration_file = rospy.get_param("~calibration_file", None)
   
    config = load_config(config_file)
    camera_calibrations = load_calibrations(rospy.get_namespace(), calibration_file)

    if rospy.get_param("~reset_cycle", False):
        spinnaker_helpers.reset_all()
        rospy.sleep(2)

    preview_sizes = dict(
        preview = rospy.get_param("~preview_width", 400),
        display = rospy.get_param("~display_width", 1200)
    )


    camera_node = CameraArrayNode(config, camera_calibrations, preview_sizes)

    try:
        camera_node.initialise()       
        camera_node.capture()

    except KeyboardInterrupt:
        pass
    camera_node.cleanup()


if __name__ == "__main__":
    main()
