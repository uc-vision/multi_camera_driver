from functools import partial
from pathlib import Path
from typing import Tuple, Union

import kornia
import rospy
import tensorrt as trt
import torch
import torch.nn.functional as F
from cached_property import cached_property
from kornia.color import CFA, RawToRgb
from nvjpeg_torch import Jpeg, JpegException
from torch import nn
from torch2trt import TRTModule, torch2trt

from spinnaker_camera_driver_helpers.image_settings import PublisherSettings

from .common import EncoderError, ImageEncoding


def compile(model, input):
  return torch2trt(model, input, 
        fp16_mode=True, log_level=trt.Logger.INFO)
        


class Sharpen(nn.Module):
  def __init__(self, kernel_size=3):
    super().__init__()
    kernel = kornia.filters.get_laplacian_kernel2d(kernel_size
      ).view(1, 1, kernel_size, kernel_size).expand(3, 1, kernel_size, kernel_size)
    
    self.register_buffer("kernel", kernel)


  def forward(self, input:torch.Tensor, factor:float):

    p = self.kernel.shape[2] // 2
    padded = F.pad(input, [p, p, p, p], mode="replicate")
    laplacian = torch.nn.functional.conv2d(padded, self.kernel, bias=None, stride=1, padding=0, groups=input.shape[1])  

    return torch.clamp(input - laplacian * factor, min=0, max=255.0)

    
class Identity(nn.Module):
  def __init__(self):
    super().__init__()

  def forward(self, input, factor):
    return input


class Debayer(nn.Module):
  def __init__(self, settings:PublisherSettings):
    super().__init__()

    layouts = {
      ImageEncoding.Bayer_RGGB8 : CFA.BG, 
      ImageEncoding.Bayer_GRBG8 : CFA.GB,
      ImageEncoding.Bayer_GBRG8 : CFA.GR,
      ImageEncoding.Bayer_BGGR8 : CFA.RG
    }

    self.settings = settings
    w, h = settings.camera.image_size

    if settings.image.resize_width > 0 and settings.image.resize_width != w:
      self.scale_factor = settings.image.resize_width / w

    self.debayer = RawToRgb(cfa=layouts[settings.camera.encoding])
    self.sharpen = Sharpen(kernel_size=3) if settings.image.is_sharpening else Identity()


  def forward(self, bayer, sharpness):
    rgb = self.debayer(bayer)

    if self.settings.image.resize_width > 0:
      rgb = F.interpolate(rgb, scale_factor=self.scale_factor, 
        mode='bilinear', align_corners=False)
    
    rgb = self.sharpen(rgb, sharpness)

    return rgb

def debayer_kornia(settings:PublisherSettings, dtype=torch.float16):
  w, h = settings.camera.image_size
  m = Debayer(settings).to(dtype=dtype, device=settings.image.device, memory_format=torch.channels_last)

  data_type = dict(dtype=dtype, device=settings.image.device)

  return compile(m, [
      torch.zeros( (1, 1, h, w), **data_type), 
      torch.tensor([settings.image.sharpen], **data_type)    
      ])


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


def cache_file(name:str, settings:PublisherSettings):
  w, h = settings.camera.image_size
  rw = settings.image.resize_width

  sharp = "_sharp" if settings.image.is_sharpening else ""
  return Path(settings.image.cache_path) / Path(f"{name}_{w}x{h}_{rw}{sharp}.pth")


class Processor(object):
  def __init__(self, settings : PublisherSettings):
    self.settings = settings
    self.jpeg = Jpeg()

    self.dtype = torch.float16
    self.create_processor(settings)

    

  
  def create_processor(self, settings:PublisherSettings):
    create_debayer = partial(debayer_kornia, settings, dtype=self.dtype)
    self.debayer = cache_model(cache_file("debayer", settings), create_debayer)


  def update_settings(self, settings):
    if any( [ settings.camera.image_size  != self.settings.camera.image_size,
        settings.image.resize_width != self.settings.image.resize_width,
        settings.image.is_sharpening != self.settings.image.is_sharpening]):
      return True

    self.settings = settings
    return False


    

  def __call__(self, raw):
    return ImageOutputs(self, raw, self.dtype, self.settings.image.device)


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
    def settings(self) -> PublisherSettings:
      return self.parent.settings
    
    @cached_property
    def cuda_rgb(self):
      with torch.inference_mode():

        batched = self.cuda_raw.view(1, 1, *self.cuda_raw.shape)
        rgb = self.parent.debayer(
          batched.to(dtype=self.dtype), 
          torch.tensor([self.settings.image.sharpen], dtype=self.dtype, device=self.device)
        )
        return rgb.to(dtype=torch.uint8)


    def encode(self, image):
      with torch.inference_mode():
        try:

          return self.parent.jpeg.encode(
              image.squeeze(0), 
              quality=self.settings.image.jpeg_quality, 
              input_format = Jpeg.RGB).numpy().tobytes()

        except JpegException as e:
          raise EncoderError(str(e))

    @cached_property 
    def compressed(self):
      return self.encode(self.cuda_rgb)


    @cached_property 
    def preview(self):
      with torch.inference_mode():      
        preview_rgb = F.interpolate(self.cuda_rgb, size=self.settings.image.preview_size)
        return self.encode(preview_rgb)
