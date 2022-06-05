#!/usr/bin/env python3


import rospy
from .camera_set import CameraSet

from dynamic_reconfigure.server import Server

from spinnaker_camera_driver_ros.cfg import CameraArrayConfig
from .camera_setters import delayed_setters, property_setters



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

      if k == "jpeg_quality":
        self.publisher.set_options(dict(quality = v))
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
      rospy.sleep(0.1)

    self.stop()


  def cleanup(self):
    self.camera_set.unregister_handlers(self.event_handlers)
    self.camera_set.cleanup()