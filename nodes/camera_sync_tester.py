"""
Point the cameras at a clock with ms precision. Check to see that the saved images show the same time across all cameras
"""

from __future__ import print_function
import rospy
import message_filters
from sensor_msgs.msg import Image, CameraInfo
import cv2
import cv_bridge
import numpy as np


def callback(*args):
    bridge = cv_bridge.CvBridge()
    print(len(args))

    images = []
    for i, image_msg in enumerate(args):
        print(image_msg.header.stamp)
        image = bridge.imgmsg_to_cv2(image_msg)
        image = cv2.resize(image, (800, 600))
        images.append(image)

    out = np.hstack((
        np.vstack(images[0:2]),
        np.vstack(images[2:4]),
        np.vstack(images[4:6])))
    cv2.imwrite("/home/josh/Downloads/{}.png".format(image_msg.header.stamp), out)
    print("*" * 80)


def main():
    rospy.init_node('camera_sync_tester', anonymous=True)
    subs = []

    for i in range(1, 7):
        subs.append(message_filters.Subscriber('cam{}/image_raw'.format(i), Image))

    ts = message_filters.TimeSynchronizer(subs, 6)
    ts.registerCallback(callback)
    rospy.spin()


if __name__ == "__main__":
    main()
