
from concurrent.futures import ThreadPoolExecutor, Future
from functools import partial, reduce
import math
import operator
from typing import List, Tuple
import numpy as np

from spinnaker_camera_driver_helpers.common import BayerPattern, CameraSettings, ImageEncoding, bayer_pattern, encoding_bits
import taichi as ti

from taichi_image import bayer, packed, tonemap


taichi_pattern = {
    BayerPattern.BGGR: bayer.BayerPattern.BGGR,
    BayerPattern.RGGB: bayer.BayerPattern.RGGB,
    BayerPattern.GBRG: bayer.BayerPattern.GBRG,
    BayerPattern.GRBG: bayer.BayerPattern.GRBG,
}



def encoding(name, camera:CameraSettings):
  enc = camera.encoding
  
  pattern = bayer_pattern(enc)
  bits = encoding_bits(enc)

  if encoding_bits(enc) not in [12, 16]:
    raise ValueError(f"Camera {name}: unsupported bits {encoding_bits(enc)} in {enc}")
  
  return pattern, bits


def resize_width(image_size, resize_width):
  if resize_width > 0:
    scale = resize_width / image_size[0]
    return scale, (resize_width, math.ceil(image_size[1] * scale))
  else:
    return 1.0, image_size

      

def resize_longest(size:Tuple[int, int], max_size: int) -> Tuple[int, int]:
  h, w = size
  scale = max_size / max(h, w)
  return int(h * scale), int(w * scale)

class TiQueue():
  executor = None
    
  @classmethod
  def queue(cls):
    if cls.executor is None:
      cls.executor = ThreadPoolExecutor(max_workers=1, 
        initializer=partial(ti.init, arch=ti.cuda, device_memory_GB=4, offline_cache=True))
    return cls.executor


  @staticmethod
  def _await_run(func, *args):
    args = [arg.result() if isinstance(arg, Future) else arg for arg in args]
    return func(*args)
      

  @staticmethod
  def run_async(func, *args) -> Future:
    return TiQueue.queue().submit(TiQueue._await_run, func, *args)

  @staticmethod
  def run_sync(func, *args):
    return TiQueue.run_async(func, *args).result()