#!/usr/bin/env python

import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image


class CaptureNode():
    def __init__(self, input_prefix, output_prefix, topics):
        self.bridge = CvBridge()

        # self.image_publisher = rospy.Publisher(topic, Image, queue_size=1)
        self.subs = [
            rospy.Subscriber("{}/{}".format(input_prefix, topic), Image, self.callback(topic), queue_size=1) for topic
            in topics
        ]

        for topic in topics:
            print("{}/{}".format(input_prefix, topic))

        self.images = {}

    def callback(self, topic):
        def f(image_msg):
            try:
                cv_image = self.bridge.imgmsg_to_cv2(image_msg, desired_encoding="passthrough")
                print(topic, image_msg.header)

            except CvBridgeError as e:
                print(e)

        return f


if __name__ == '__main__':

    image_topics = ["cam{}/image_raw".format(i) for i in range(6)]
    rospy.init_node('capture_node')

    node = CaptureNode("camera_array", "capture", image_topics)
    while not rospy.core.is_shutdown():
        rospy.rostime.wallsleep(0.5)
