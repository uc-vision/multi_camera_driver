from __future__ import annotations
import traceback
from typing import Callable
import weakref
from beartype import beartype
import rospy2 as rospy

from queue import Queue
from threading import Thread


class WorkQueue():

  def __init__(self, name, run:Callable, num_workers=1, max_size=None):
        
    self.queue = Queue(max_size or num_workers)
    self.workers = None
    self.num_workers = num_workers
    
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
    rospy.loginfo(f"Waiting for {self.name}: threads {self.workers}")
    
    for worker in self.workers:
      self.queue.put(None)
    
    for worker in self.workers:
      worker.join()
    print(f"Done {self.name}: thread {self.workers}")


  def start(self):
    assert self.workers is None

    self.workers = [Thread(target = self.run_worker) 
                    for _ in range(self.num_workers)]
    for worker in self.workers:
      worker.start()

      