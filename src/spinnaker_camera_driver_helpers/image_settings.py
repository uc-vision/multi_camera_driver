from dataclasses import dataclass
import numpy as np

from .common import CameraSettings

class InvalidOption(Exception):
  pass

@dataclass
class ImageSettings:
  device : str = 'cuda:0'
  resize_width: float = 0.0

  preview_size : int = 200
  jpeg_quality : int = 94

  # Tonemapping parameters
  tone_gamma: float = 1.0
  tone_intensity : float = 1.0

  light_adapt : float = 1.0
  color_adapt : float = 1.0

  
  # Moving average to smooth intensity scaling
  ema_alpha : float = 0.2

  @property
  def is_sharpening(self):
    return self.sharpen > 0

  @staticmethod
  def settings():
    """ Settings able to be changed dynamically """

    return ['preview_size', 'jpeg_quality',  'resize_width', 
            'tone_gamma', 'tone_intensity', 'light_adapt', 
            'color_adapt', 'ema_alpha']
    

  @property
  def is_resizing(self):
    return self.resize_width > 0.0

  def __post_init__(self):
    self.preview_size = int(self.preview_size)
    self.jpeg_quality = int(np.clip(self.jpeg_quality, 1, 100))

