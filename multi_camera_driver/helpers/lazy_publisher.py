import os
from typing import Callable, Optional

import rospy2 as rospy
import rclpy



def get_node() -> rclpy.node.Node:
  return rospy._node

class LazyPublisher:
  """ Setup lazy publishers for a bunch of topics for inputs from a single source """

  def __init__(self, topics, queue_size=4, 
               name=None, on_register:Optional[Callable]=None, on_unregister:Optional[Callable]=None):
    
    super().__init__()
    self.topics = {}
    self.on_register = on_register or (lambda: [])
    self.on_unregister = on_unregister or (lambda _: None)

    for topic_name, (topic_type, f) in topics.items():
      full_topic = rospy.get_namespace() + (f'{name}/{topic_name}'
                                            if name is not None else topic_name)
      publisher = rospy.Publisher(full_topic,
            topic_type,
            queue_size=queue_size
        )     

      self.subscriptions = []

      rospy.logdebug(
          f'Created publisher {topic_name}: {full_topic} {topic_type}')
      self.topics[full_topic] = (f, publisher)

      

  def find_topic(self, topic_name):
    if topic_name in self.topics:
      return self.topics[topic_name]
    else:
      rospy.logwarn(
          f'LazyPublisher.find_topic: not found {topic_name} in {self.topics.keys()}'
      )
      return None


  def any_subscribed(self):
    for _, (_, pub) in self.topics.items():
      if pub.get_num_connections() > 0:
        return True
    return False


  def publish(self, data, header):
    for _, (getter, pub) in self.topics.items():
      if pub.get_num_connections() > 0:
        x = getter(data)
        x.header = header
        pub.publish(x)

