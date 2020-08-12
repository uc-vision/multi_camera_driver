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

    np.set_printoptions(precision=5, suppress=True)
    resize = rospy.get_param("~resize", None)
    

    left = rospy.get_param("~left")
    right = rospy.get_param("~right")

    topic = rospy.get_param("~topic")

    processor = StereoPublisher(topic, left, right, resize=resize)
    
    rospy.spin()



if __name__ == "__main__":
    main()

