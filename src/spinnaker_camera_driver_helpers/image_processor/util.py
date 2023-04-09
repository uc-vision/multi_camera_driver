
from concurrent.futures import ThreadPoolExecutor, Future
from typing import Tuple
import numpy as np

from spinnaker_camera_driver_helpers.common import BayerPattern, ImageEncoding, bayer_pattern, encoding_bits
import taichi as ti

from taichi_image import bayer, packed


taichi_pattern = {
    BayerPattern.BGGR: bayer.BayerPattern.BGGR,
    BayerPattern.RGGB: bayer.BayerPattern.RGGB,
    BayerPattern.GBRG: bayer.BayerPattern.GBRG,
    BayerPattern.GRBG: bayer.BayerPattern.GRBG,
}


# This is needed because torch doesn't have unsigned integer types
@ti.kernel
def load_16f_kernel(image: ti.types.ndarray(ti.u16),
                    out: ti.types.ndarray(ti.f16)):
  for I in ti.grouped(image):
    out[I] = ti.cast(ti.cast(image[I], ti.f32) / 65535.0, ti.f16)


def load_16f(image: np.ndarray):
  out = ti.ndarray(shape=tuple(image.shape))
  load_16f_kernel(image, out)
  return out


      

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
        initializer=ti.init, initargs=[ti.cuda])
    return cls.executor


  @staticmethod
  def _await_run(func, *args):
    args = [arg.result() if isinstance(arg, Future) else arg for arg in args]
    return func(*args)
      

  @staticmethod
  def run_async(func, *args):
    return TiQueue.queue().submit(TiQueue._await_run, func, *args)

  @staticmethod
  def run_sync(func, *args):
    return TiQueue.run_async(func, *args).result()