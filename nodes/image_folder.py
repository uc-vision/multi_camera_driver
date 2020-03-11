#!/usr/bin/env python
from __future__ import print_function

import cv2
from turbojpeg import TurboJPEG

import rospy
from os import path

from cv_bridge import CvBridge
from std_msgs.msg import Header
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo
from geometry_msgs.msg import TransformStamped

from camera_geometry_ros.conversions import camera_info_msg
from camera_geometry.import_calibration import import_cameras
from camera_geometry import image_utils, json, util

from tf.transformations import quaternion_from_matrix
import tf2_ros

def load_calibration(calibration_file):
    calib_data = json.load_json(calibration_file)
    return import_cameras(calib_data)

jpeg = TurboJPEG()

def load_image(filename):
    with open(filename, 'rb') as in_file:
        return jpeg.decode(in_file.read())


def defer(f, args):
    thread = Thread(target=f, args=args)
    thread.start()

    return thread


def load_image_set(filenames):
    return [load_image(filename) for filename in filenames]
 


class ImagePublisher(object):
    def __init__(self, name, camera):
        super(ImagePublisher, self).__init__()

        self.bridge = CvBridge()
        self.name = name

        self.image_publisher = rospy.Publisher("{}/image_raw".format(self.name), Image, queue_size=4)
        self.info_publisher = rospy.Publisher("{}/cam_info".format(self.name), CameraInfo, queue_size=4)

        self.cam_info = CameraInfo() if camera is None \
             else camera_info_msg(camera)

    def publish(self, image, timestamp, encoding="rgb8"):

        header = Header()
        header.stamp = timestamp
        header.frame_id = self.name

        image_msg = self.bridge.cv2_to_imgmsg(image, encoding=encoding)
        image_msg.header = header
        
        self.cam_info.header = header

        self.info_publisher.publish(self.cam_info)
        self.image_publisher.publish(image_msg)



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
    

def publish_extrinsics(extrinsics):

    stamp = rospy.Time.now()
    broadcaster = tf2_ros.StaticTransformBroadcaster()

    for frame_id, transform in extrinsics.items():
        transform_msg = TransformStamped()
  
        transform_msg.header.stamp = stamp
        transform_msg.header.frame_id = transform.parent
        transform_msg.child_frame_id = frame_id

        transform_msg.transform.translation = transform.translation
        transform_msg.transform.rotation = quaternion_from_matrix(util.expand_identity(transform.rotation))

        broadcaster.sendTransform(transform_msg)

def main():


    rospy.init_node('image_folder_node', anonymous=False)
    looping = rospy.get_param("~looping", False)
    frequency = rospy.get_param("~frequency", 5)

    image_path = rospy.get_param("~image_path")
    calibration_file = rospy.get_param("~calibration_file", None)

    cameras = {}
    extrinsics = []

    if calibration_file is not None:
        cameras, extrinsics = load_calibration(calibration_file)

    publish_extrinsics(extrinsics)
    
    camera_dirs, image_sets =  image_utils.find_image_dirs(image_path)
    print("Found camera directories {} with {} matching images".format(str(camera_dirs), len(image_sets)))

    def publisher(dir):
        name = path.basename(dir)
        return ImagePublisher(name, cameras.get(name))

    publishers = [publisher(dir) for dir in camera_dirs]


    try:
        publish(image_sets, publishers, looping=looping, frequency=frequency)
    except KeyboardInterrupt:
        pass
    

if __name__ == "__main__":
    main()
