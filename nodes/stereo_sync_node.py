#!/usr/bin/env python
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image

from threading import Thread
import os
import message_filters
import traceback
import errno    


from camera_geometry_ros.conversions import rectified_info_msg
from camera_geometry import image_utils, json, stereo_pair, import_calibration


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



# def process_frame(message):
#     bridge = CvBridge()
#     colour = cv2.cvtColor(img, cv2.COLOR_BAYER_BG2BGR)

    
def defer(f, args):
    thread = Thread(target=f, args=args)
    thread.start()
    return thread


class StereoProcessor(object):

    def __init__(self, info):
        self.info = info


    def frame_callback(self, *frames):
        pass




def load_calibration(calibration_file):
    calib_data = json.load_json(calibration_file)
    cameras = import_calibration.import_stereo_pairs(calib_data)

    return cameras


def stereo_synchroniser(pair, camera_node):
    left = pair.cameras[0].name
    right = pair.cameras[1].name

    topics = ["/{}/{}/image_raw".format(camera_node, camera) for camera in [left, right]]
    subscribers = [message_filters.Subscriber(topic, Image) for topic in topics]

    info_msg = stereo_info_msg(pair)
    processor = StereoProcessor(info_msg)

    sync = message_filters.TimeSynchronizer(subscribers, 3)
    sync.registerCallback(processor.frame_callback)
 
 

def main():

    rospy.init_node('stereo_sync_node', anonymous=True)

    calibration_file = rospy.get_param("~calibration_file")
    camera_node = rospy.get_param("~camera_node")

    stereo_pairs = load_calibration(calibration_file)

    synchronisers = [stereo_synchroniser(pair, camera_node) 
        for pair in stereo_pairs]

    
    rospy.spin()



if __name__ == "__main__":
    main()

    print("Capture images ended")
