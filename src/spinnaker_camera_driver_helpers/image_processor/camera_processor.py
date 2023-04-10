
import numpy as np
import torch

import taichi as ti
from taichi_image import bayer, interpolate, tonemap, packed, types

from spinnaker_camera_driver_helpers.common import CameraSettings
from spinnaker_camera_driver_helpers.image_settings import ImageSettings
from .util import encoding, load_16f_kernel, resize_width, taichi_pattern


class CameraProcessor(object):
  def __init__(self, name:str, settings:ImageSettings, camera:CameraSettings, dtype = ti.f16, device = "cuda:0"):
    self.settings = settings
    self.camera = camera
    self.dtype = dtype
    self.device = device

    self.name = name
    self.pattern, self.bits = encoding(name, camera)
    self.min_max = np.array([0, 1])

    self.reinhard_kernel = tonemap.reinhard_kernel(self.dtype, ti.u8)
    self.scale_offset_kernel = tonemap.scale_offset_kernel(self.dtype)
    self.bilinear_kernel = interpolate.bilinear_kernel(self.dtype)

    self.bayer_to_rgb = bayer.bayer_to_rgb_kernel(taichi_pattern[self.pattern], self.dtype)
    self.load_kernel = (load_16f_kernel if self.bits == 16  
                         else packed.decode12_kernel(self.dtype, scaled=True))
    
    self.min_max_kernel = tonemap.min_max_kernel(dtype=self.dtype)
    self.create_buffers()


  def update_settings(self, settings:ImageSettings, camera:CameraSettings):
    if self.camera.image_size != camera.image_size or self.settings.resize_width != settings.resize_width:
      self.create_buffers()

    self.settings = settings
    self.camera = camera

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


  def load_image(self, image_raw):
    self.load_kernel(image_raw, self.bayer16.view(-1))
    self.bayer_to_rgb(self.bayer16, self.rgb)
    
    if self.scale != 1:
      self.bilinear_kernel(self.rgb, self.resized, scale=ti.math.vec2(self.scale, self.scale))
    self.min_max = np.array(self.min_max_kernel(self.resized))
    
  
  def outputs(self, min_max):
    preview_scale, preview_size = resize_width(self.output_size, self.settings.preview_size)
    output = torch.empty((self.output_size[1], self.output_size[0], 3), dtype=torch.uint8, device=self.device)

    self.scale_offset_kernel(self.resized, 
        scale=1/(min_max[1] - min_max[0]), offset=-min_max[0])
    self.reinhard_kernel(self.resized, output, self.settings.gamma, 
        self.settings.intensity, self.settings.light_adapt, self.settings.color_adapt)
    
    preview = interpolate.resize_bilinear(output, preview_size, preview_scale)
    return output, preview