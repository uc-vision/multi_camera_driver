#!/usr/bin/env python3

import rospy
from std_msgs.msg import Header, Float64
import numpy as np

def main():
  rospy.init_node('ping_client', anonymous=False)
  pub = rospy.Publisher("/ping", Header, queue_size=10)

  offsets = []
  returns = []

  def on_pong(msg):
    ret = (rospy.Time.now() - msg.header.stamp).to_sec()
    returns.append(ret)
    offsets.append(msg.data)

    med_offset = np.median(offsets)
    med_return = np.median(returns)

    diff = (med_offset - med_return) / 2
    ping_time = (med_offset + med_return) / 2

    rospy.loginfo(f"Time offset: {diff:.3f} Ping: {ping_time:.3f}")

  sub = rospy.Subscriber("/pong", Float64, on_pong)
  rate = rospy.Rate(10)

  while not rospy.is_shutdown():
    msg = Header()
    msg.stamp = rospy.Time.now()
    pub.publish(msg)

    rate.sleep()


if __name__ == "__main__":
  main()