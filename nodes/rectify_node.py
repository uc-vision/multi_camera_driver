#!/usr/bin/env python
import rospy
import cv2
import numpy as np

from threading import Thread
import os
import message_filters
import traceback
import errno    

import tf2_ros

from camera_geometry_ros.conversions import rectified_info_msg, transform_from_msg, camera_from_msg
from camera_geometry import image_utils, json, stereo_pair, import_calibration

from sensor_msgs.msg import CameraInfo
from maara_msgs.msg import StereoCameraInfo


def stereo_info_msg(pair):
    assert isinstance(pair, stereo_pair.StereoPair)

    left, right = pair.rectified

    msg = StereoCameraInfo()
    msg.left_info = rectified_info_msg(left)
    msg.right_info = rectified_info_msg(right)

    msg.T_left_right = pair.translation
    msg.R_left_right = pair.rotation

    msg.Q = left.disparity_to_depth

    return msg
  
def defer(f, args):
    thread = Thread(target=f, args=args)
    thread.start()
    return thread

class StereoPublisher(object):

    def __init__(self, name, left, right, queue_size=4):

        self.buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.buffer)

        self.frames = (left, right)
        self.name = name

        topics = ["{}/camera_info".format(camera) for camera in [left, right]]

        self.publisher = rospy.Publisher("{}/stereo_info".format(self.name), StereoCameraInfo, queue_size=queue_size)

        self.info_publishers = [
            rospy.Publisher("{}/{}/camera_info".format(self.name, camera), CameraInfo, queue_size=queue_size)
                for camera in ['left', 'right']
        ]

        subscribers = [message_filters.Subscriber(topic, CameraInfo) for topic in topics]

        sync = message_filters.TimeSynchronizer(subscribers, queue_size=queue_size)
        sync.registerCallback(self.frame_callback)


    def frame_callback(self, left_info, right_info):
        try:
            msg = self.buffer.lookup_transform(self.frames[1], self.frames[0], rospy.Time())
            child_frame, transform = transform_from_msg(msg)
            
            left = camera_from_msg(left_info)
            right = camera_from_msg(right_info, extrinsic = transform.extrinsic)

            pair = stereo_pair.rectify_pair(left, right)
                     
            stereo_info = stereo_info_msg(pair)
            header = stereo_info.header
            header.stamp = left_info.header.stamp
            header.frame_id = self.name

            for msg in [stereo_info, stereo_info.left_info, stereo_info.right_info]:
                msg.header = header

            self.publisher.publish(stereo_info)
            
            for publisher, msg in zip(self.info_publishers, [stereo_info.left_info, stereo_info.right_info]):
                publisher.publish(msg)

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            pass    

def main():

    rospy.init_node('stereo_node', anonymous=True)

    stereo_topic = rospy.get_param("~topic")
    left, right = rospy.get_param("~left"),  rospy.get_param("~right")

    processor = StereoPublisher(stereo_topic, left, right)
    
    rospy.spin()



if __name__ == "__main__":
    main()

