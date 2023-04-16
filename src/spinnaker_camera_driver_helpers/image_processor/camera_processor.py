
from typing import List
import numpy as np
import torch

import taichi as ti
from taichi_image import bayer, interpolate, tonemap, packed, types

from spinnaker_camera_driver_helpers.common import CameraSettings
from spinnaker_camera_driver_helpers.image_settings import ImageSettings
from .util import ema, encoding,  load_16u_kernel, resize_width, taichi_pattern
from py_structs.numpy import shape_info



class CameraProcessor(object):
  def __init__(self, name:str, settings:ImageSettings, camera:CameraSettings, dtype = ti.f32, device = "cuda:0"):
    self.settings = settings
    self.camera = camera
    self.dtype = dtype
    self.device = device

    self.name = name
    self.pattern, self.bits = encoding(name, camera)

    
    self.reinhard_kernel = tonemap.reinhard_kernel(self.dtype, ti.u8)
    self.bilinear_kernel = interpolate.bilinear_kernel(self.dtype)
    self.metering_kernel = tonemap.metering_kernel(self.dtype)

    self.linear_kernel = tonemap.linear_kernel_for(self.dtype, ti.u8)

    self.bayer_to_rgb = bayer.bayer_to_rgb_kernel(taichi_pattern[self.pattern], self.dtype)
    # self.load_kernel = packed.decode12_kernel(self.dtype, scaled=True)
    self.load_kernel = load_16u_kernel(dtype, 1/65536.0)

    

    self.min_max_kernel = tonemap.min_max_kernel(dtype=self.dtype)
    self.create_buffers()


  def _update_settings(self, settings:ImageSettings, camera:CameraSettings):
    if self.camera.image_size != camera.image_size or self.settings.resize_width != settings.resize_width:
      self.create_buffers()

    self.settings = settings
    self.camera = camera


  def update_settings(self, settings:ImageSettings):
    self._update_settings(settings, self.camera)

  def update_camera(self, settings:CameraSettings):
    self._update_settings(self.settings, settings)


  def create_buffers(self):
    w, h = self.camera.image_size

    self.bayer16 = torch.zeros((h, w), dtype=types.ti_to_torch[self.dtype], device=self.device)
    self.rgb = torch.zeros((h, w, 3), dtype=types.ti_to_torch[self.dtype], device=self.device)

    if self.settings.resize_width > 0:
      self.scale, self.output_size = resize_width(self.camera.image_size, self.settings.resize_width)
      self.resized = torch.zeros((self.output_size[1], self.output_size[0], 3), dtype=types.ti_to_torch[self.dtype], device=self.device)
      
      # self.rescaled = torch.zeros((self.output_size[1], self.output_size[0], 3), dtype=types.ti_to_torch[self.dtype], device=self.device)
    else:
      self.resized = self.rgb
      self.scale = 1
      self.output_size = self.camera.image_size


  def load_image(self, image_raw):
    # print(shape_info(image_raw))
    # self.load_kernel(image_raw, self.bayer16.view(-1))
    self.load_kernel(image_raw.reshape(self.bayer16.shape), self.bayer16)
    
    self.bayer_to_rgb(self.bayer16, self.rgb)
    if self.scale != 1:
      self.bilinear_kernel(self.rgb, self.resized, scale=ti.math.vec2(self.scale, self.scale))
    
    # print(self.resized.mean(),  torch.clip(self.resized, min=1e-4).log().mean())

  def bounds(self):
    return self.min_max_kernel(self.resized)

  def metering(self, bounds):
    return self.metering_kernel(self.resized, *bounds)




  def outputs(self, bounds, metering):
    preview_scale, preview_size = resize_width(self.output_size, self.settings.preview_size)
    output = torch.empty((self.output_size[1], self.output_size[0], 3), dtype=torch.uint8, device=self.device)


    if self.settings.tone_mapping == "reinhard":

      tonemap.rescale_kernel(self.resized, *bounds)
      print(type(metering))

      self.reinhard_kernel(self.resized, output, metering, self.settings.tone_gamma, 
          self.settings.tone_intensity, self.settings.light_adapt, self.settings.color_adapt)
      

    elif self.settings.tone_mapping == "linear":      

      self.linear_kernel(self.resized, output,
          *bounds, gamma=self.settings.tone_gamma, scale_factor=255)
    else:
      raise NotImplementedError(self.settings.tone_mapping)
    

    preview = interpolate.resize_bilinear(output, preview_size, preview_scale)
    return output, preview
  
