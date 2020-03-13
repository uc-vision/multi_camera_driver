#!/usr/bin/env python

import rospy
import numpy as np

from threading import Thread
import os
import traceback
import errno    

from spinnaker_camera_driver_helpers.common import StereoPublisher

def main():

    rospy.init_node('stereo_node', anonymous=True)

    stereo_topic = rospy.get_param("~topic")
    left, right = rospy.get_param("~left"),  rospy.get_param("~right")

    processor = StereoPublisher(stereo_topic, left, right)
    
    rospy.spin()



if __name__ == "__main__":
    main()

