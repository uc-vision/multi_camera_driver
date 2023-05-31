#!/usr/bin/env python3

import rospy
from std_msgs.msg import Header
from spinnaker_camera_driver_ros.msg import FloatStamped

def main():
  rospy.init_node('ping_client', anonymous=False)


  def on_ping(msg):
    offset = (rospy.Time.now() - msg.stamp).to_sec()
    header = Header(stamp=rospy.Time.now())

    msg = FloatStamped(header, offset)

    pub.publish(msg)

  sub = rospy.Subscriber("/ping", Header, on_ping)
  pub = rospy.Publisher("/pong", FloatStamped, queue_size=10)

  rospy.spin()

if __name__ == "__main__":
  main()