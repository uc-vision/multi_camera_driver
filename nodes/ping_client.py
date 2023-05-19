#!/usr/bin/env python3

import rospy
from std_msgs.msg import Header, Float64

def main():
  rospy.init_node('ping_client', anonymous=False)
  sub = rospy.Subscriber("/ping", Header, on_ping)
  pub = rospy.Publisher("/pong", Float64, queue_size=10)

  def on_ping(msg):
    offset = (rospy.Time.now() - msg.stamp).to_sec()
    msg = Float64(offset)
    msg.header.stamp = rospy.Time.now()
    pub.publish(msg)


  rospy.spin()

if __name__ == "__main__":
  main()