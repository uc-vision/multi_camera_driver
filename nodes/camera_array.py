#!/usr/bin/env python3
from __future__ import print_function

from traceback import format_exc
from functools import partial

import math
import PySpin
from structs.struct import struct
import rospy

from dynamic_reconfigure.server import Server

from spinnaker_camera_driver_helpers import spinnaker_helpers
from spinnaker_camera_driver_helpers.image_handler import ImageHandler
from spinnaker_camera_driver_helpers.sync_handler import SyncHandler

from spinnaker_camera_driver_helpers.publisher import ImageSettings
from spinnaker_camera_driver_ros.cfg import CameraArrayConfig

from spinnaker_camera_driver_helpers.camera_setters import delayed_setters, property_setters
from spinnaker_camera_driver_helpers.config import load_calibrations, load_config, publish_extrinsics

import gc



class ImageEventHandler(PySpin.ImageEventHandler):
  def __init__(self, on_image):
    super(ImageEventHandler, self).__init__()
    self.on_image = on_image

  def OnImageEvent(self, image):
    try:
        self.on_image(image)
    except:
      rospy.logerr(f"ImageEventHandler exception: {format_exc()}")




class CameraArrayNode(object):
    def __init__(self, publisher, camera_serials, camera_settings,
       master_id=None, free_running=False):

        self.config = {}
        self.pending_config = {}
        self.started = False
        self.free_running = free_running
        self.publisher = publisher

        assert master_id is None or master_id in camera_serials
        self.master_name = camera_serials.get(master_id, None)
        self.camera_dict = spinnaker_helpers.find_cameras(camera_serials)  # camera_name -> camera

        rospy.loginfo("Initialising cameras")
        self.camera_info = {k : self.init_camera(camera, k, camera_settings) 
          for k, camera in self.camera_dict.items()}
        self.event_handlers = self.add_handlers()
        
        self.reconfigure_srv = Server(CameraArrayConfig, self.reconfigure_callback)
        rospy.loginfo(f"{len(self.camera_dict)} Cameras initialised")


    @property
    def master(self):
      if self.master_name is not None:
          return self.camera_dict[self.master_name]      


    def set_property(self, key, value, setter):
        try:
            rospy.loginfo(f"set_property {key}: {value}")
            for k, camera in self.camera_dict.items():
                setter(camera, value, self.camera_info[k])      
        except PySpin.SpinnakerException as e:
            rospy.loginfo(f"set_property: {key} {value} {e} ")

    
    def set_config_properties(self, config):
        for k, v in config.items():           
            if k == "groups":
                continue

            if k == "jpeg_quality":
                self.publisher.set_option('quality', v)
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
      self.set_config_properties(config)  
      return config


    def trigger(self):
        assert self.master_name is not None
        try:
            spinnaker_helpers.trigger(self.master)
        except PySpin.SpinnakerException as e:
            rospy.logerr("Error triggering: " + str(e))


    def start(self):
        rospy.loginfo("Begin acquisition")

        self.publisher.start()
        for k, camera in self.camera_dict.items():
           camera.BeginAcquisition()
           assert spinnaker_helpers.validate_streaming(camera),\
             f"Camera {k} did not begin streaming"

        self.started = True

    def stop(self):
      if self.started:
      
        rospy.loginfo("End acquisition")
        self.publisher.stop()

        for camera in self.camera_dict.values():
            camera.EndAcquisition()

        self.started = False

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
        assert not self.started 
        self.start()

        self.trigger()
        while not rospy.is_shutdown():           
            self.update_pending()    
            self.trigger()           

            rate = self.config.get("max_framerate", math.inf)
            rospy.sleep(1.0/rate if rate > 0 else 0)

        self.stop()

    def capture_free(self):
        assert not self.started 
        self.start()

        while not rospy.is_shutdown():
          self.update_pending()
          rospy.sleep(0.1)
  
        self.stop()



    def init_camera(self, camera : PySpin.Camera, camera_name, camera_settings):
      try:
          camera.Init()
          assert spinnaker_helpers.validate_init(camera),\
              f"Camera {camera_name} did not initialise"

          is_master = camera_name == self.master_name

          info = struct(
            connection_speed = spinnaker_helpers.get_current_speed(camera),
            serial =  spinnaker_helpers.get_camera_serial(camera),
            time_offset_sec = spinnaker_helpers.camera_time_offset(camera),
            is_triggered = self.master_name is None, 
            is_master = is_master,
            is_free_running = self.free_running and is_master or self.master is None 
          )

          rospy.loginfo(f"{camera_name}: {info}") 

          spinnaker_helpers.set_camera_settings(camera, camera_settings)

          if self.master_name is not None:
            spinnaker_helpers.trigger_master(camera, self.free_running)\
              if info.is_master else spinnaker_helpers.trigger_slave(camera)

          return info
      except PySpin.SpinnakerException as e:
          rospy.logerr(f"Could not initialise camera {camera_name}: {str(e)}")



    def add_handlers(self):      

      def register_handler(k, camera):
        on_image = lambda image: self.publisher.publish(
          image, camera_name=k, camera_info=self.camera_info[k])

        handler = ImageEventHandler(on_image)
        camera.RegisterEventHandler(handler)
        return handler
      
      return {k : register_handler(k, camera) 
        for k, camera in self.camera_dict.items() }
       


    def cleanup(self):
        rospy.loginfo("Stopping cameras")
        
        for k, camera in self.camera_dict.items():
            camera.UnregisterEventHandler(self.event_handlers[k])

            spinnaker_helpers.load_defaults(camera)
            camera.DeInit()
        
def main():
    rospy.init_node('camera_array_node', anonymous=False)

    rospy.loginfo("Starting")
    config_file = rospy.get_param("~config_file")

    
    calibration_file = rospy.get_param("~calibration_file", None)
   
    config = load_config(config_file)
    camera_names = config["camera_aliases"].values()
    camera_calibrations = load_calibrations(calibration_file, camera_names)

    if rospy.get_param("~reset_cycle", False):
        spinnaker_helpers.reset_all()
        rospy.sleep(2)

    image_settings = ImageSettings(
        preview_size = rospy.get_param("~preview_width", 400),
        encoding = config.get("encoding", "bayer_rggb8"),
        device = rospy.get_param("~device", 'cuda:0'),
        quality = 90
    )
 
    system = PySpin.System.GetInstance()
    publisher = ImageHandler(camera_names, image_settings)
    
    camera_node = CameraArrayNode(
      publisher = publisher,
      camera_serials = config["camera_aliases"],
      camera_settings = config["camera_settings"],
      master_id = config.get("master", None),
      free_running=True
    )

    publish_extrinsics(camera_calibrations, camera_names)

    try:
        camera_node.capture()
    except KeyboardInterrupt:
        pass
    except PySpin.SpinnakerException as e:
        rospy.logerr("Error capturing:" + str(e))

    publisher.stop()
    camera_node.stop()

    camera_node.cleanup()
    del camera_node
    gc.collect()

    system.ReleaseInstance()


if __name__ == "__main__":
    main()
