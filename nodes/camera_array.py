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

import numpy as np
import math
import PySpin

from dynamic_reconfigure.server import Server
import sys
import traceback

from spinnaker_camera_driver_helpers import spinnaker_helpers
from spinnaker_camera_driver_helpers.common import *
from spinnaker_camera_driver_helpers.publisher import SyncPublisher, SyncHandler
from spinnaker_camera_driver_ros.cfg import CameraArrayConfig

import gc
from cached_property import cached_property


ImageEvent = getattr(PySpin, 'ImageEventHandler', None) or getattr(PySpin, 'ImageEvent')

class ImageEventHandler(ImageEvent):
    def __init__(self, publisher, camera_name, time_offset_sec):
        super(ImageEventHandler, self).__init__()
        self.publisher = publisher
        self.camera_name = camera_name
        self.time_offset_sec = time_offset_sec

    def OnImageEvent(self, image):
        try:
            self.publisher.process_camera_image(
                self.camera_name, image, self.time_offset_sec)
        except:
            traceback.print_exc(file=sys.stdout)
            sys.exit(1)


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


def set_black_level(camera, value):
    """Set the target grey value. If 0 set to auto"""
    node_map = camera.GetNodeMap()
    return spinnaker_helpers.try_set_float(node_map, "BlackLevel", value)


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
    black_level = set_black_level,
    ev_comp = set_ev_comp,
    gamma = set_gamma,
    max_framerate = set_internal
)

delayed_setters = dict(    
    binning = set_binning
)

class CameraArrayNode(object):
    def __init__(self, config):
        self.system = PySpin.System.GetInstance()
        assert config is not None

        self.handler = handler
        self.camera_settings = config["camera_settings"]

        self.camera_serials = config["camera_aliases"] # serial -> alias
        self.master_id = config.get("master", None)   

        self.free_running = config.get("free_running", False)
        self.camera_dict = spinnaker_helpers.find_cameras(self.camera_serials)  # alias -> camera
        assert self.master_id is None or self.master_id in self.camera_serials

        self.config = {}
        self.pending_config = {}
        
        self.event_handlers = []
        self.initialised = []
        
        self.cameras_initialised = False
        self.started = False

        self.reconfigure_srv = Server(CameraArrayConfig, self.reconfigure_callback)
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
            if k == "groups":
                pass
            elif k == "jpeg_quality":
                self.handler.set_quality(v)
            else:
                self.set_camera_property(k, v)
            
    
    def set_camera_property(self, k, v):
        setter = property_setters.get(k, None) or delayed_setters.get(k, None)       
        if setter is None:
            rospy.logerr(f"reconfigure: no property {k}")
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

      if self.started:
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
        else self.capture_software()

    def capture_software(self):
        assert len(self.initialised) > 0 and not self.started 
        self.start()

        last_time = rospy.Time.now()
        self.trigger()
        while not rospy.is_shutdown():           
            self.update_pending()    
            self.trigger()           

            time = rospy.Time.now()
            rate = self.config.get("max_framerate", math.inf)

            if rate > 0:
              rospy.sleep(1.0/rate)
            else:
              rospy.sleep(0)
            last_time = time


        self.stop()

    def capture_free(self):
        assert len(self.initialised) > 0 and not self.started 
        self.start()

        while not rospy.is_shutdown():
          self.update_pending()
          rospy.sleep(0.1)
  
        self.stop()


    def init_camera(self, camera, camera_name):
      try:
          camera.Init()

          nodemap_tldevice = camera.GetTLDeviceNodeMap()
          serial = PySpin.CStringPtr(nodemap_tldevice.GetNode('DeviceSerialNumber'))

          rospy.loginfo("Initialising: {} {}".format(camera_name, serial.GetValue()))

          spinnaker_helpers.set_camera_settings(camera, self.camera_settings)

          time_offset_sec = spinnaker_helpers.camera_time_offset(camera)
          rospy.loginfo(f"{camera_name} time offset: {time_offset_sec} (s)")

          info = spinnaker_helpers.get_camera_info(camera)
          rospy.loginfo(f"{camera_name}: {info}")

          if self.master_id is not None:
              spinnaker_helpers.enable_triggering(camera, master=camera_name == self.master_name, free_running=self.free_running)

          return self.publish_camera(camera_name, camera, time_offset_sec)
      except PySpin.SpinnakerException as e:
          rospy.logerr("Could not initialise camera {}: {}".format(camera_name, str(e)))


    def publish_camera(self, camera_name, camera, time_offset_sec):      
      event_handler = ImageEventHandler(self.handler, camera_name, time_offset_sec)

      if getattr(camera, 'RegisterEvent', None):
          camera.RegisterEvent(event_handler)
      else:
          camera.RegisterEventHandler(event_handler)
          
      return event_handler


    def cleanup(self):
        del self.initialised
        del self.event_handlers
        gc.collect()

        rospy.loginfo("Stopping cameras")

        # Why are we doing this here?
        # Otherwise the camera can't be examined in spinview (trigger mode etc.)
        for camera in self.camera_dict.values():
            spinnaker_helpers.load_defaults(camera)
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
    camera_names = config["camera_aliases"].values()
    camera_calibrations = load_calibrations(calibration_file, camera_names)

    if rospy.get_param("~reset_cycle", False):
        spinnaker_helpers.reset_all()
        rospy.sleep(2)

    publisher = SyncPublisher(camera_names, camera_calibrations, 
        preview_size = rospy.get_param("~preview_width", 400),
        encoding     = config.get("encoding", "bayer_rggb8")
    )

    handler = SyncHandler(publisher, camera_names, 
      sync_threshold_msec = rospy.get_param("~sync_threshold_msec", 2))

    camera_node = CameraArrayNode(config, handler)



    try:
        camera_node.initialise()  
        publisher.publish_extrinsics()
        camera_node.capture()

    except KeyboardInterrupt:
        pass

    publisher.stop()
    camera_node.cleanup()


if __name__ == "__main__":
    main()
