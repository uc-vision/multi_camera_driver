#!/usr/bin/env python3


import gc
from dataclasses import fields

import rospy
from dynamic_reconfigure.server import Server

from spinnaker_camera_driver_ros.cfg import CameraArrayConfig


from .camera_set import CameraSet
from .camera_setters import delayed_setters, property_setters
from .publisher import ImageSettings


class CameraArrayNode(object):
  def __init__(self, publisher, camera_set : CameraSet):

    self.camera_set = camera_set

    self.config = {}
    self.pending_config = {}
    self.publisher = publisher

    self.camera_set = camera_set
    
    self.event_handlers = self.camera_set.register_publisher(publisher)
    self.reconfigure_srv = Server(CameraArrayConfig, self.reconfigure_callback)


  def set_config_properties(self, config):
    for k, v in config.items():
      if k == "groups":
        continue

      if k in fields(ImageSettings):
        self.publisher.set_option(k, v)
      else:
        self.set_camera_property(k, v)

    if not self.started:
      self.check_image_sizes()
      

  def check_image_sizes(self):
    image_sizes = self.camera_set.get_image_sizes()
    for k, image_size in image_sizes.items():
      info = self.camera_set.camera_info[k]
      if info.image_size != image_size:
        rospy.loginfo(f"Camera {k} updated image size {image_size}")
        info.image_size = image_size
        self.publisher.set_camera_options(k, dict(image_size=image_size))


  def set_camera_property(self, k, v):
    setter = property_setters.get(k, None) or delayed_setters.get(k, None)
    if setter is None:
      rospy.logerr(f"reconfigure: no property {k}")
      return

    if self.config.get(k, None) != v and setter is not None:
      if self.started and k in delayed_setters:
        self.pending_config[k] = v
      else:
        self.camera_set.set_property(k, v, setter)
        self.config[k] = v

  def reconfigure_callback(self, config, _):
    self.set_config_properties(config)
    return config


  @property
  def started(self):
    return self.camera_set.started

  def start(self):
    assert not self.started
    self.publisher.start()
    self.camera_set.start()


  def stop(self):
    if self.camera_set.started:
      self.publisher.stop()
      gc.collect(0)
      
      self.camera_set.stop()


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
    self.start()

    while not rospy.is_shutdown():
      self.update_pending()
      self.publisher.report_recieved()
      rospy.sleep(0.2)

    self.stop()


  def cleanup(self):
    self.camera_set.unregister_handlers(self.event_handlers)
    self.camera_set.cleanup()