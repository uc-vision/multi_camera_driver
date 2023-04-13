from __future__ import annotations
import traceback
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
      try:
        data = self.queue.get()
        while data is not None:
          self.run(data)
          data = self.queue.get()
      except Exception as e:
        trace = traceback.format_exc()
        rospy.logerr(trace)
        rospy.logerr(f"Exception in {self.name}: {e}")
        
        

  def stop(self):
    rospy.loginfo(f"Waiting for {self.name}: thread {self.worker}")
    
    self.queue.put(None)
    self.worker.join()
    print(f"Done {self.name}: thread {self.worker}")


  def start(self):
    assert self.worker is None

    self.worker = Thread(target = self.run_worker)
    self.worker.start()

      