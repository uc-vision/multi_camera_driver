#!/usr/bin/env python3

from dataclasses import fields, replace
import gc
import rospy
from .sync_handler import SyncHandler
from .image_settings import ImageSettings, InvalidOption

from .camera_set import CameraSet

import rospy
from dynamic_reconfigure.server import Server

from spinnaker_camera_driver_ros.cfg import CameraArrayConfig

from .camera_set import CameraSet
from . import camera_setters 


class CameraArrayNode(object):
  delayed_setters = ["binning"]
  
  def __init__(self, publisher:SyncHandler, camera_set:CameraSet):

    self.camera_set = camera_set
    self.settings = publisher.image_settings

    self.config = {}
    self.pending_config = {}
    self.publisher = publisher
    self.event_handlers = None

    self.camera_set = camera_set  
    self.reconfigure_srv = Server(CameraArrayConfig, self.reconfigure_callback)

  def update_settings(self, settings:ImageSettings):
    try:
      return self.publisher.update_settings(settings)
    except InvalidOption as e:
      rospy.logwarn(e)

  def check_image_sizes(self):
    assert not self.started, "Cannot update image sizes while cameras are started"

    image_sizes = self.camera_set.get_image_sizes()
    for camera_name, image_size in image_sizes.items():
      info = self.camera_set.camera_settings[camera_name]
      if info.image_size != image_size:
        rospy.loginfo(f"Camera {camera_name} updated image size {image_size}")

        info = replace(info, image_size = image_size)
        self.publisher.update_camera(camera_name, info)

  def set_camera_property(self, k, v):
      assert not (k in self.delayed_setters and self.started),\
        f"Cannot set {k} while cameras are started"
      rospy.loginfo(f"Setting {k}={v}")

      if k in camera_setters.property_setters:
        setter = camera_setters.property_setters[k]
        self.camera_set.set_property(k, v, setter)

      self.config[k] = v

  def update_setting(self, k, v):
    settings = self.settings
    
    if k in ImageSettings.settings():
      settings = replace(settings, **{k:v})
      if self.update_settings(settings) and self.started:
        return True
      
    self.settings = settings
    return False


  def reconfigure_callback(self, config, _):
    for k, v in config.items():
      if k == "groups":
        continue

      if self.config.get(k, None) == v:
        continue

      delayed = self.update_setting(k, v) or k in self.delayed_setters 
      if self.started and delayed:
        self.pending_config[k] = v
      else:
        self.set_camera_property(k, v)


    if not self.started:
      self.check_image_sizes()

    return config

  @property
  def started(self):
    return self.camera_set.started

  def start(self):
    assert not self.started
    self.event_handlers = self.camera_set.register_publisher(self.publisher)

    self.publisher.start()
    self.camera_set.start()


  def stop(self):
    if self.camera_set.started:
      self.camera_set.unregister_handlers(self.event_handlers)
      self.publisher.stop()

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
        self.update_setting(k, v)


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