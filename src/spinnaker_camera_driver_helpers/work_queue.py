from __future__ import annotations
from typing import Callable
import weakref
from beartype import beartype
import rospy

from queue import Queue
from threading import Thread


class WorkQueue():

  def __init__(self, name, run:Callable, max_size=0):    
    self.queue = Queue(max_size)
    self.worker = None
    self.name = name
    self.run = run

    
  def enqueue(self, data):
      return self.queue.put( data )
  
  def run_worker(self):
      data = self.queue.get()
      while data is not None:
        self.run(data)
        data = self.queue.get()

  def stop(self):
    rospy.loginfo(f"Waiting for {self.name}: thread {self.worker}")
    
    self.queue.put(None)
    self.worker.join()
    print(f"Done {self.name}: thread {self.worker}")


  def start(self):
    assert self.worker is None

    self.worker = Thread(target = self.publish_worker)
    self.worker.start()

      