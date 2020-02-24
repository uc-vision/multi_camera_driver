#!/usr/bin/env python
"""ROS node for spinnaker cameras.

Copyright (c) 2019. Sam Schofield. This file is subject to the 3-clause BSD
license, as found in the LICENSE file in the top-level directory of this
distribution and at https://github.com/sds53/spinnaker_camera_driver_ros/LICENSE.
No part of spinnaker_camera_driver_helpers, including this file, may be copied, modified,
propagated, or distributed except according to the terms contained in the
LICENSE file.
"""

from __future__ import print_function, division

import yaml
from threading import Lock

import PySpin
import rospy
from cv_bridge import CvBridge
from mavros_msgs.msg import CamIMUStamp
from mavros_msgs.srv import CommandTriggerControl
from sensor_msgs.msg import Image

from spinnaker_camera_driver_helpers import spinnaker_helpers
from dynamic_reconfigure.server import Server
from spinnaker_camera_driver_ros.cfg import CameraIMUSettingsConfig


class ImageImuStampEventHandler(PySpin.ImageEvent):
    def __init__(self, cam, topic, cam_imu_offset=0):
        super(ImageImuStampEventHandler, self).__init__()
        self.node_map = cam.GetNodeMap()
        self.topic = topic
        self.lock = Lock()
        self.bridge = CvBridge()
        self.image_publisher = rospy.Publisher(topic, Image, queue_size=1)
        self.stamp_buffer = []
        self.image_buffer = []
        self.first_message = True
        self.cam_imu_offset = cam_imu_offset
        self.sync_sub = rospy.Subscriber("mavros/cam_imu_sync/cam_imu_stamp", CamIMUStamp, self.stamp_cb, queue_size=10)
        del cam
    #     self.reconfigure_srv = Server(CameraIMUSettingsConfig, self.reconfigure_callback)
    #
    # def reconfigure_callback(self, config, _):
    #     return config

    def sync_and_publish(self):
        # TODO: I think we can do this more efficiently
        with self.lock:
            if len(self.stamp_buffer) == 0 or len(self.image_buffer) == 0:
                return

            image_buffer = []
            for image_message in self.image_buffer:
                image_sequence = image_message.header.seq
                stamp_buffer = []
                match_found = False
                for stamp_message in self.stamp_buffer:
                    stamp_sequence = stamp_message.frame_seq_id

                    if image_sequence == stamp_sequence:
                        image_message.header.stamp = stamp_message.frame_stamp + rospy.Duration(self.cam_imu_offset)
                        self.image_publisher.publish(image_message)
                        match_found = True
                    else:
                        stamp_buffer.append(stamp_message)

                self.stamp_buffer = stamp_buffer
                if not match_found:
                    image_buffer.append(image_message)
            self.image_buffer = image_buffer
            self.trim_buffers()

    def trim_buffers(self):
        max_buff_size = 10
        if len(self.image_buffer) > max_buff_size:
            self.image_buffer = self.image_buffer[len(self.image_buffer) - max_buff_size:]

        if len(self.stamp_buffer) > max_buff_size:
            self.stamp_buffer = self.stamp_buffer[len(self.stamp_buffer) - max_buff_size:]

    def stamp_cb(self, sync_message):
        rospy.logdebug_throttle(1, "Stamp received")
        with self.lock:
            self.stamp_buffer.append(sync_message)
        self.sync_and_publish()

    def OnImageEvent(self, image):
        rospy.logdebug_throttle(1, "Image received")
        if image.IsIncomplete():
            print('Image incomplete with image status %d ...' % image.GetImageStatus())
        else:
            image_data = image.GetNDArray()
            # FrameCounter always 0 in chunk data so use the nodemap instead
            # TODO: handle camera models that don't have FrameCounter.
            try:
                frame_counter = PySpin.CValuePtr(self.node_map.GetNode('ChunkFrameCounter'))
                sequence_number = int((frame_counter.ToString()))
            except:
                rospy.logdebug_throttle(1, "Couldn't get frame counter")
                return

            image.Release()
            image_msg = self.bridge.cv2_to_imgmsg(image_data, encoding="bayer_rggb8")
            image_msg.header.stamp = rospy.Time.now()

            image_msg.header.seq = sequence_number
            with self.lock:
                self.image_buffer.append(image_msg)
            self.sync_and_publish()


def init_camera(camera, image_topic, settings, cam_imu_offset):
    camera.Init()
    spinnaker_helpers.set_camera_settings(camera, settings)
    event_handler = ImageImuStampEventHandler(camera, image_topic, cam_imu_offset)
    camera.RegisterEvent(event_handler)
    return camera, event_handler


class CameraArrayNode(object):
    def __init__(self, config=None):
        # TODO: actually handle when config is None
        self.bridge = CvBridge()
        self.service_proxy = rospy.ServiceProxy("/mavros/cmd/trigger_control", CommandTriggerControl)
        self.system = PySpin.System.GetInstance()
        self.camera_list = self.system.GetCameras()
        self.camera_dict = spinnaker_helpers.camera_list_to_dict(self.camera_list)
        if config is None:
            config = dict()
        self.cam_imu_offset = config.get("cam-imu-offset", 0)
        self.camera_settings = config.get("camera_settings", None)
        self.camera_aliases = config.get("camera_aliases", None)
        print("{} cameras found.".format(self.camera_list.GetSize()))

    def start_triggering(self):
        try:
            self.service_proxy.call(trigger_enable=True, sequence_reset=True)
        except rospy.ServiceException:
            print("Could not reset triggering")

    def stop_triggering(self):
        try:
            self.service_proxy.call(trigger_enable=False)
        except rospy.ServiceException:
            print("Could not reset triggering")

    def start(self):
        print("Starting")
        if self.camera_list.GetSize() == 0:
            print("No cameras found")
            return

        event_handlers = []
        for serial, camera in self.camera_dict.items():
            if self.camera_aliases is None:
                alias = "cam_{}".format(serial)
            else:
                alias = self.camera_aliases.get(serial, "cam_{}".format(serial))
            image_topic = "{}/image_raw".format(alias)
            camera, event_handler = init_camera(camera, image_topic, self.camera_settings, self.cam_imu_offset)
            event_handlers.append(event_handler)
            del camera  # TODO: Why do we actually do this?

        # Making sure the triggering has stopped before starting the cameras.
        self.stop_triggering()
        print("Start cameras")
        for camera in self.camera_list:
            camera.BeginAcquisition()
        rospy.sleep(1)
        print("Start triggering")
        self.start_triggering()
        print("Running")
        rospy.spin()
        print("Acquisition ended")
        for camera in self.camera_list:
            camera.EndAcquisition()
        del camera
        # TODO: UnregisterEventHandler?

    def stop(self):
        print("Stopping camera")
        for camera in self.camera_list:
            camera.DeInit()
        del camera
        del self.camera_dict
        self.camera_list.Clear()
        self.system.ReleaseInstance()


def main():
    import argparse
    rospy.init_node('camera_array_node', anonymous=True, log_level=rospy.INFO)

    parser = argparse.ArgumentParser(description='Node to run visual inertial rig')
    parser.add_argument('config_file', type=str, help="The file to convert")
    args = parser.parse_args()
    config_file = args.config_file
    if config_file is not None:
        with open(config_file) as config_file:
            config = yaml.load(config_file, Loader=yaml.Loader)
            camera_node = CameraArrayNode(config=config)
    else:
        camera_node = CameraArrayNode()
    try:
        camera_node.start()
    except KeyboardInterrupt:
        pass
    camera_node.stop()


if __name__ == "__main__":
    main()
