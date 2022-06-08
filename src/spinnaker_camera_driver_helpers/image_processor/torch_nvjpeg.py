from functools import partial
from nvjpeg_torch import Jpeg, JpegException
import torch
from pathlib import Path

from .common import EncoderError

import torch.nn.functional as F
from ..publisher import ImageSettings

from cached_property import cached_property
from torch import nn

import tensorrt as trt
from torch2trt import torch2trt, TRTModule
from kornia.color import CFA, RawToRgb

import rospy


def debayer_module(settings, dtype=torch.float16):
    from debayer import DebayerSplit, Layout

    layouts = dict(
      bayer_rggb8 = Layout.RGGB, 
      bayer_grbg8 = Layout.GRBG,
      bayer_gbrg8 = Layout.GBRG,
      bayer_bggr8 = Layout.BGGR
    )
    assert settings.encoding in layouts

    debayer = DebayerSplit(layouts[settings.encoding]
      ).to(dtype=dtype, device=settings.device, memory_format=torch.channels_last)

    return debayer

def compile(model, input):
  return torch2trt(model, input, 
        fp16_mode=True,   log_level=trt.Logger.INFO)
        
def debayer_kornia(settings, dtype=torch.float16):

  layouts = dict(
    bayer_rggb8 = CFA.BG, 
    bayer_grbg8 = CFA.GB,
    bayer_gbrg8 = CFA.GR,
    bayer_bggr8 = CFA.RG
  )
  w, h = settings.image_size

  m = RawToRgb(cfa=layouts[settings.encoding])
  return compile(m, [torch.zeros(1, 1, h, w, 
      dtype=dtype, device=settings.device)])


def cache_model(cache_file, create_model):
  model = None

  if Path(cache_file).is_file():
    rospy.logdebug(f"Attempting to load cache file {cache_file}")
    try:
      m = TRTModule()
      m.load_state_dict(torch.load(cache_file))
      model = m
      
    except (IOError, KeyError):
      pass

  if model is None:
    rospy.logdebug(f"Cache failed (or does not exist) creating model")
    model = create_model()

    rospy.logdebug(f"Saving to {cache_file}")
    torch.save(model.state_dict(), cache_file)

  return model


def cache_file(name, settings):
  w, h = settings.image_size
  return Path(settings.cache_path) / Path(f"{name}_{w}x{h}.pth")


class Processor(object):
  def __init__(self, settings : ImageSettings):
    self.settings = settings
    self.jpeg = Jpeg()

    self.dtype = torch.float16

    create_debayer = partial(debayer_kornia, settings, dtype=self.dtype)
    self.debayer = cache_model(cache_file("debayer", settings), create_debayer)


  def __call__(self, raw):
    return ImageOutputs(self, raw, self.dtype, self.settings.device)


class ImageOutputs(object):
    def __init__(self, parent, raw, dtype, device):
        self.parent = parent
        self.raw = raw
        self.dtype = dtype
        self.device = device

    @cached_property
    def cuda_raw(self):
      return torch.from_numpy(self.raw).to(device=self.device)

    @property 
    def settings(self) -> ImageSettings:
      return self.parent.settings

    @cached_property
    def cuda_rgb(self):
      with torch.inference_mode():
        batched = self.cuda_raw.view(1, 1, *self.cuda_raw.shape)
        rgb = self.parent.debayer(batched.to(dtype=self.dtype))
        return rgb.to(dtype=torch.uint8)

    def encode(self, image):
      with torch.inference_mode():
        try:

          return self.parent.jpeg.encode(
              image.squeeze(0), 
              quality=self.settings.quality,
              input_format = Jpeg.RGB).numpy().tobytes()
        except JpegException as e:
          raise EncoderError(str(e))

    @cached_property 
    def compressed(self):
      return self.encode(self.cuda_rgb)


    @cached_property 
    def preview(self):
      with torch.inference_mode():      
        preview_rgb = F.interpolate(self.cuda_rgb, size=self.settings.preview_size)
        return self.encode(preview_rgb)
