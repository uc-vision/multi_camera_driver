#!/usr/bin/env python3

import rospy2 as rospy

from rcl_interfaces.msg import SetParametersResult


from dataclasses import replace
import gc
import traceback
from beartype import beartype

from .image_settings import ImageSettings
from .camera_set import CameraSet
from std_srvs.srv import Empty
from pydispatch import Dispatcher

from std_msgs.msg import Bool
from rclpy.qos import QoSProfile, DurabilityPolicy
QOS_LATCHING = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)

class CameraArrayNode(Dispatcher):
  delayed_setters = ["binning"]
  _events_ = ["on_image_settings", "on_camera_settings", "on_update"]
  
  @beartype
  def __init__(self, camera_set: CameraSet, image_settings: ImageSettings = ImageSettings()):

    self.camera_set = camera_set

    self.config = {}
    self.pending_config = {}
    self.image_settings = image_settings

    self.require_resync = False

    rospy._node.add_on_set_parameters_callback(self.reconfigure_callback)


    self.start_cameras_service()
    self.stop_cameras_service()
    self.cameras_started_pub = rospy._node.create_publisher(Bool, '/cameras_started', 
                                                            qos_profile=QOS_LATCHING)
    
    for camera_name, info in camera_set.camera_settings.items():
      rospy.loginfo(f"{camera_name}: {info}")

  def resync(self):
    self.require_resync = True

  def set_property(self, k, v):
      assert not (k in self.delayed_setters and self.started),\
        f"Cannot set {k} while cameras are started"
      rospy.loginfo(f"Setting {k}={v}")

      if k in self.camera_set.camera_properties():
        if self.camera_set.set_property(self.config, k, v):
          self.config[k] = v

      elif hasattr(self.image_settings, k):
        self.image_settings = replace(self.image_settings, **{k: v})    
        self.emit("on_image_settings", self.image_settings)    
      else:
        rospy.logwarn(f"Unknown property {k}")
      

      return self.config

  def start_cameras_service(self):
    def start_cameras(req, resp):
      if not self.started:
        self.start()
      return resp
    return rospy._node.create_service(Empty, '~start_cameras', start_cameras)
  
  def stop_cameras_service(self):
    def stop_cameras(req, resp):
      if self.started:
        self.stop()
      return resp
    return rospy._node.create_service(Empty, '~stop_cameras', stop_cameras)


  def reconfigure_callback(self, params):
    try:
      for param in params:
        k, v = param.name, param.value
        if self.config.get(k, None) == v:
          continue

        if self.started and k in self.delayed_setters:
          self.pending_config[k] = v
        else:
          self.set_property(k, v)

      if self.camera_set.check_camera_settings():
        self.emit("on_camera_settings", self.camera_set.camera_settings)
    except Exception as e:
      trace = traceback.format_exc()
      print(f"{trace}")
      rospy.logerr(f"Error while applying settings: {e}")

      raise e

    return SetParametersResult(successful=True)

  @property
  def started(self):
    return self.camera_set.started

  def start(self):
    assert not self.started
    self.camera_set.register_handlers()
    self.camera_set.start()
    self.cameras_started_pub.publish(Bool(data=True))


  def stop(self):
    if self.camera_set.started:
      self.camera_set.unregister_handlers()

      gc.collect(0)
      self.camera_set.stop()
      self.cameras_started_pub.publish(Bool(data=False))


  def update_pending(self):
    if len(self.pending_config) == 0:
      return 
      
    if self.started:
      rospy.loginfo(f"Applying pending settings")

      self.stop()
      for k, v in self.pending_config.items():
        self.set_property(k, v)

      self.start()
    else:
      config = self.set_property(self.pending_config)
      #self.reconfigure_srv.update_configuration(config)


    self.pending_config = {}

  def capture(self, auto_start = True):
    if auto_start:
      self.start()

    while not rospy.is_shutdown():
      self.update_pending()
      self.emit("on_update")

      if self.require_resync:
        self.camera_set.stop()
        rospy.sleep(1.0)
        self.camera_set.start()
        self.require_resync = False  
      
      rospy.sleep(0.2)

    self.stop()


  def cleanup(self):
    self.stop()
    self.camera_set.cleanup()
