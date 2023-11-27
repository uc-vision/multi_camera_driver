from dataclasses import dataclass
from enum import Enum
from multi_camera_driver.helpers.camera_params import ToneMapper, Transform
import numpy as np

from .common import CameraSettings

class InvalidOption(Exception):
  pass



@dataclass
class ImageSettings:
  device : str = 'cuda:0'
  resize_width: int = 2048

  preview_size : int = 200
  jpeg_quality : int = 94

  # Tonemapping parameters
  tone_gamma: float = 1.0
  tone_intensity : float = 1.0

  light_adapt : float = 1.0
  color_adapt : float = 1.0

  # linear reinhard
  tone_mapping: int = ToneMapper.linear.value

  # none rotate_90 rotate_180 rotate_270 transpose flip_horiz flip_vert 
  transform : int = Transform.none.value
  
  # Moving average to smooth intensity scaling over time
  moving_average : float = 0.02

  @property
  def is_sharpening(self):
    return self.sharpen > 0

  @property
  def is_resizing(self):
    return self.resize_width > 0.0

  def __post_init__(self):
    self.preview_size = int(self.preview_size)
    self.jpeg_quality = int(np.clip(self.jpeg_quality, 1, 100))

    

