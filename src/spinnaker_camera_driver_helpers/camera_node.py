#!/usr/bin/env python3

from dataclasses import fields, replace
import gc
from beartype import beartype
import rospy
from .sync_handler import SyncHandler
from .image_settings import ImageSettings, InvalidOption

from .camera_set import CameraSet

import rospy
from dynamic_reconfigure.server import Server

from spinnaker_camera_driver_ros.cfg import CameraArrayConfig

from .camera_set import CameraSet
from . import camera_setters 

from pydispatch import Dispatcher


class CameraArrayNode(Dispatcher):
  delayed_setters = ["binning"]
  _events_ = ["on_image_settings"]
  
  @beartype
  def __init__(self, camera_set:CameraSet, image_settings:ImageSettings):

    self.camera_set = camera_set

    self.config = {}
    self.pending_config = {}
    self.event_handlers = None
    self.image_settings = image_settings

    self.reconfigure_srv = Server(CameraArrayConfig, self.reconfigure_callback)


  def set_property(self, k, v):
      assert not (k in self.delayed_setters and self.started),\
        f"Cannot set {k} while cameras are started"
      rospy.loginfo(f"Setting {k}={v}")

      if k in self.camera_set.camera_properties():
        self.camera_set.set_property(k, v)
      elif hasattr(self.image_settings, k):
        self.image_settings = replace(self.image_settings, **{k: v})    
        self.emit("on_image_settings", self.image_settings)    
      else:
        rospy.logwarn(f"Unknown property {k}")
      
      self.config[k] = v


  def reconfigure_callback(self, config, _):
    for k, v in config.items():
      if k == "groups":
        continue

      if self.config.get(k, None) == v:
        continue

      if self.started and k in self.delayed_setters:
        self.pending_config[k] = v
      else:
        self.set_property(k, v)

    self.camera_set.check_camera_settings()
    return config

  @property
  def started(self):
    return self.camera_set.started

  def start(self):
    assert not self.started
    self.event_handlers = self.camera_set.register_publisher(self.publisher)
    self.camera_set.start()


  def stop(self):
    if self.camera_set.started:
      self.camera_set.unregister_handlers(self.event_handlers)

      gc.collect(0)
      self.camera_set.stop()


  def update_pending(self):
    if len(self.pending_config) == 0:
      return 
      
    if self.started:
      rospy.loginfo(f"Applying pending settings")

      self.stop()
      for k, v in self.pending_config.items():
        self.set_camera_property(k, v)

      self.start()
    else:
      self.set_camera_property(self.pending_config)

    self.pending_config = {}

  def capture(self):
    self.start()

    while not rospy.is_shutdown():
      self.update_pending()
      self.publisher.report_recieved()
      rospy.sleep(0.2)

    self.stop()


  def cleanup(self):
    self.stop()
    self.camera_set.cleanup()