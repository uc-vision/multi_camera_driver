#!/usr/bin/env python3
from __future__ import print_function

from traceback import format_exc

import PySpin
from structs.struct import struct
import rospy

from std_msgs.msg import String
from dynamic_reconfigure.server import Server

from spinnaker_camera_driver_helpers import spinnaker_helpers
from spinnaker_camera_driver_helpers.image_handler import ImageHandler
from spinnaker_camera_driver_helpers.sync_handler import SyncHandler

from spinnaker_camera_driver_helpers.publisher import ImageSettings
from spinnaker_camera_driver_ros.cfg import CameraArrayConfig

from spinnaker_camera_driver_helpers.camera_setters import delayed_setters, property_setters
from spinnaker_camera_driver_helpers.config import import_calibrations, load_calibrations, load_config, publish_extrinsics, write_calibration

import gc

import tf2_ros



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
    def __init__(self, publisher, camera_serials, camera_settings, master_id=None):

        self.config = {}
        self.pending_config = {}
        self.started = False
        self.publisher = publisher

        assert (master_id is None) or master_id in camera_serials
        self.master_name = camera_serials.get(master_id, None)
        self.camera_dict = spinnaker_helpers.find_cameras(camera_serials)  # camera_name -> camera

        rospy.loginfo(f"Initialising cameras: {camera_serials}")
        rospy.loginfo(f"Triggering: {'Disabled' if self.master_name is None else 'Enabled'}")
        
        self.camera_info = {k : self.init_camera(camera, k, camera_settings) 
          for k, camera in self.camera_dict.items()}
        self.event_handlers = self.add_handlers()
        
        self.reconfigure_srv = Server(CameraArrayConfig, self.reconfigure_callback)
        rospy.loginfo(f"{len(self.camera_dict)} Cameras initialised")


    @property
    def camera_names(self):
      return list(self.camera_dict.values())

    @property
    def camera_ids(self):
      return list(self.camera_dict.keys())

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
            is_free_running = is_master or self.master is None 
          )

          rospy.loginfo(f"{camera_name}: {info}") 

          spinnaker_helpers.set_camera_settings(camera, camera_settings)

          if self.master_name is not None:
            spinnaker_helpers.trigger_master(camera, True)\
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
    tracking_frame = rospy.get_param("tracking_frame", None)
   
    config = load_config(config_file)
    camera_names = config["camera_aliases"].values()
    calib = load_calibrations(calibration_file, camera_names, tracking_frame)

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
    
    master_id = config.get("master", None)
    HandlerType = ImageHandler if master_id is None else SyncHandler
    publisher = HandlerType(camera_names, image_settings, calibration=calib)
    
    broadcaster = tf2_ros.StaticTransformBroadcaster()
    publish_extrinsics(broadcaster, calib, camera_names)


    def on_recalibrated(msg):
      try:
        calib = import_calibrations(msg.data, camera_names, tracking_frame)

        publisher.update_calibration(calib.cameras)
        publish_extrinsics(broadcaster, calib, camera_names)

        write_calibration(calibration_file, msg.data)
      except:
        rospy.logerr(f"on_recalibrated exception: {format_exc()}")


    recalibrate_sub = rospy.Subscriber("recalibrated", String, on_recalibrated)

    camera_node = CameraArrayNode(
      publisher = publisher,
      camera_serials = config["camera_aliases"],
      camera_settings = config["camera_settings"],
      master_id = master_id,
    )


    try:
        camera_node.capture()
    except KeyboardInterrupt:
        pass
    except PySpin.SpinnakerException as e:
        rospy.logerr("Error capturing:" + str(e))

    recalibrate_sub.unregister()

    publisher.stop()
    camera_node.stop()

    camera_node.cleanup()
    del camera_node
    gc.collect()

    system.ReleaseInstance()


if __name__ == "__main__":
    main()
