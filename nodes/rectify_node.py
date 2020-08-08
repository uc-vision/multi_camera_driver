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

    resize = rospy.get_param("~resize")
    rectify = rospy.get_param("~rectify")
 
    processor = StereoPublisher("stereo", "left", "right", resize=resize, rectify=rectify)
    
    rospy.spin()



if __name__ == "__main__":
    main()

