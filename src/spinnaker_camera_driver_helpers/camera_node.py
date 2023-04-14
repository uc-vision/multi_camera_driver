#!/usr/bin/env python3

from dataclasses import fields, replace
import gc
import traceback
from beartype import beartype
import rospy

from spinnaker_camera_driver_helpers.diagnostics import CameraDiagnosticUpdater
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
  _events_ = ["on_image_settings", "on_camera_settings", "on_update"]
  
  @beartype
  def __init__(self, camera_set:CameraSet, image_settings:ImageSettings):

    self.camera_set = camera_set

    self.config = {}
    self.pending_config = {}
    self.image_settings = image_settings

    self.diagnostics = CameraDiagnosticUpdater(camera_set.camera_settings)
    camera_set.bind(on_settings=self.diagnostics.on_camera_info, on_image=self.diagnostics.on_image)


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
    try:
      for k, v in config.items():
        if k == "groups":
          continue

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

    return config

  @property
  def started(self):
    return self.camera_set.started

  def start(self):
    assert not self.started
    self.camera_set.register_handlers()
    self.camera_set.start()


  def stop(self):
    if self.camera_set.started:
      self.camera_set.unregister_handlers()

      gc.collect(0)
      self.camera_set.stop()


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
      self.set_property(self.pending_config)

    self.pending_config = {}

  def capture(self):
    self.start()

    while not rospy.is_shutdown():
      self.update_pending()
      self.diagnostics.reset()
      
      rospy.sleep(0.2)

    self.stop()


  def cleanup(self):
    self.stop()
    self.camera_set.cleanup()