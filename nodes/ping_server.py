#!/usr/bin/env python3

import rospy
from std_msgs.msg import Header
from spinnaker_camera_driver_ros.msg import FloatStamped

import numpy as np

def main():
  rospy.init_node('ping_server', anonymous=False)
  pub = rospy.Publisher("/ping", Header, queue_size=10)

  window = rospy.Duration(secs = rospy.get_param("~window", 100))

  entries = []


  def on_pong(msg):
    now = rospy.Time.now()
    ret = (now - msg.header.stamp).to_sec()
    entries.append(dict(time=now, pong_time=ret, ping_time=msg.data))

    while len(entries) > 0 and ((entries[0]["time"] + window) < now ):
      entries.pop(0)


    med_offset = np.median([e["ping_time"] for e in entries])
    med_return = np.median([e["pong_time"] for e in entries])
    diff = (med_offset - med_return) / 2
    ping_time = (med_offset + med_return) / 2

    rospy.loginfo(f"Time offset {len(entries)}: {diff:.4f} Ping: {ping_time:.4f}")

  sub = rospy.Subscriber("/pong", FloatStamped, on_pong)
  rate = rospy.Rate(10)

  while not rospy.is_shutdown():
    msg = Header()
    msg.stamp = rospy.Time.now()
    pub.publish(msg)

    rate.sleep()


if __name__ == "__main__":
  main()
