from functools import partial
from pathlib import Path
from typing import Tuple, Union

import torch
import torch.nn.functional as F
from cached_property import cached_property
from torch import nn

from nvjpeg_torch import Jpeg, JpegException
import numpy as np


from spinnaker_camera_driver_helpers.image_settings import PublisherSettings
from .common import EncoderError, ImageEncoding, BayerPattern, bayer_pattern, encoding_bits
import taichi as ti
from taichi_image import bayer, packed, interpolate

from concurrent.futures import ThreadPoolExecutor


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


def load_16f(image: np.ndarray, device: torch.device):
  out = torch.empty(tuple(image.shape), dtype=torch.float16, device=device)
  load_16f_kernel(image, out)
  return out


def encoding(settings):
  encoding = settings.camera.encoding

  pattern = bayer_pattern(encoding)
  bits = encoding_bits(encoding)

  if not bits in [12, 16]:
    raise ValueError(f"Unsupported bits for hdr_jxl {bits} in {encoding}")

  return pattern, bits


def resize_longest(size:Tuple[int, int], max_size: int) -> Tuple[int, int]:
  h, w = size
  scale = max_size / max(h, w)
  return int(h * scale), int(w * scale)


class Processor(object):

  def __init__(self, settings: PublisherSettings):
    self.dtype = torch.float16
    self.update_settings(settings)
    self.jpeg = Jpeg()

  
  def run(func, *args):
    if Processor._executor is None:
      Processor._executor = ThreadPoolExecutor(max_workers=1, initializer=ti.init, initargs=[ti.cuda])

    return Processor._executor.submit(func, *args)


  def update_settings(self, settings):
    self.settings = settings
    self.encoding = encoding(settings)

    return False

  def __call__(self, raw):
    return ImageOutputs(raw, self.jpeg, self.settings)


class ImageOutputs(object):

  def __init__(self, raw, jpeg, settings):
    self.raw = raw
    self.jpeg = jpeg

    self.device = settings.image.device
    self.image_size = settings.camera.image_size
    self.preview_size = resize_longest(self.image_size, settings.image.preview_size)

    self.pattern, self.bits = encoding(settings)
    self.jpeg_quality = settings.image.jpeg_quality


  def execute(self, func, *args):
    return Processor.run(func, *args)


  @cached_property
  def cuda_raw(self):
    if self.bits == 16:
      return load_16f(self.raw, self.device)
    else:
      return packed.decode12(self.raw, dtype=ti.f16, scaled=True)

  @cached_property
  def cuda_rgb(self):
    pattern = taichi_pattern[self.pattern]
    return bayer.bayer_to_rgb(self.cuda_raw, pattern)

  def encode(self, image):
    try:
      return self.jpeg.encode(image.squeeze(0),
                              quality=self.jpeg_quality,
                              input_format=Jpeg.RGBI).numpy().tobytes()

    except JpegException as e:
      raise EncoderError(str(e))

  @cached_property
  def compressed(self):
    return self.encode((self.cuda_rgb * 255.0).to(torch.uint8))

  @cached_property
  def preview(self):


    preview_rgb = interpolate.resize_bilinear(self.cuda_rgb,
                                              size=self.preview_size, dtype=ti.uint8)
    return self.encode(preview_rgb)
