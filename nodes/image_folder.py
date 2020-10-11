#!/usr/bin/env python3
from __future__ import print_function

import cv2
from turbojpeg import TurboJPEG
import numpy as np

from spinnaker_camera_driver_helpers.common import *

import rospy
from os import path

jpeg = TurboJPEG()

def load_image(filename):
    with open(filename, 'rb') as in_file:
        return jpeg.decode(in_file.read())


def load_image_set(filenames):
    return [load_image(filename) for filename in filenames]
 

def publish(image_sets, publishers, frequency=5, looping=False):
    frame = 0

    print("Publishing images...")
    rate = rospy.Rate(frequency)

    while(frame == 0 or looping):
        for image_set in image_sets:
            timestamp = rospy.Time.now()

            images = load_image_set(image_set)
            for publisher, image in zip(publishers, images):
                publisher.publish(image, timestamp)

            if rospy.is_shutdown():
                return

            rate.sleep()
        frame = frame + 1


def main():

    rospy.init_node('image_folder_node', anonymous=False)
    looping = rospy.get_param("~looping", False)
    frequency = rospy.get_param("~frequency", 5)

    image_path = rospy.get_param("~image_path")

    calibration_file = rospy.get_param("~calibration_file", None)

    cameras, extrinsics, stereo_pairs = load_calibration(calibration_file)
    broadcaster = publish_extrinsics(extrinsics)
    
    camera_dirs, image_sets =  image_utils.find_image_dirs(image_path)
    print("Found camera directories {} with {} matching images".format(str(camera_dirs), len(image_sets)))

    def publisher(dir):
        name = path.basename(dir)
        return CalibratedPublisher(name, cameras.get(name), encoding='bgr8')

    publishers = [publisher(dir) for dir in camera_dirs]


    stereo_processors = [StereoPublisher(name, left, right) 
        for name, (left, right) in stereo_pairs.items()]


    try:
        publish(image_sets, publishers, looping=looping, frequency=frequency)
    except KeyboardInterrupt:
        pass
    

if __name__ == "__main__":
    main()
