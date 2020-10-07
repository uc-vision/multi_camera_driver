#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from std_srvs.srv import Empty, EmptyResponse
import message_filters

class TriggerSimulator():

    def __init__(self, publishers):
        self.triggered = False
        self.publishers = publishers

    def srv_trigger(self, message):
        rospy.loginfo("Triggered")
        self.triggered = True
        return EmptyResponse()

    def listener_image_topics(self, *image_messages):
        if self.triggered:
            for i, image_message in enumerate(image_messages):
                self.publishers[i].publish(image_message)
            self.triggered = False
        else:
            pass


def main():
    rospy.init_node('camera_trigger_simulator', anonymous=True)
    image_topics = rospy.get_param("~image_topics")
    triggered_image_topics = rospy.get_param("~triggered_image_topics")
    print(image_topics)
    print(triggered_image_topics)

    assert len(image_topics) == len(triggered_image_topics)

    subscribers = [message_filters.Subscriber(image_topic, Image) for image_topic in image_topics]
    sync = message_filters.TimeSynchronizer(subscribers, 3)

    publishers = [rospy.Publisher(triggered_image_topic, Image) for triggered_image_topic in triggered_image_topics]

    triggerSimulator = TriggerSimulator(publishers)
    sync.registerCallback(triggerSimulator.listener_image_topics)
    rospy.Service("~/trigger", Empty, triggerSimulator.srv_trigger)

    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        return
    except KeyboardInterrupt:
        return

if __name__ == '__main__':
    main();
