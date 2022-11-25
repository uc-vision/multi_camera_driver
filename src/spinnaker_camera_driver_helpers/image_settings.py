from dataclasses import dataclass
import numpy as np

from .camera_set import CameraSettings

class InvalidOption(Exception):
  pass

@dataclass
class ImageSettings:
  cache_path : str 

  device : str = 'cuda:0'
  queue_size : int = 4

  preview_size : int = 200
  jpeg_quality : int = 90
  image_backend : str = 'turbo_jpeg'

  resize_width: float = 0.0
  sharpen: float = 0.0

  @property
  def is_sharpening(self):
    return self.sharpen > 0

  @staticmethod
  def settings():
    """ Settings able to be changed dynamically """
    return ['preview_size', 'jpeg_quality', 'image_backend', 'resize_width', 'sharpen']
    

  @property
  def is_resizing(self):
    return self.resize_width > 0.0

  def __post_init__(self):
    self.preview_size = int(self.preview_size)
    self.jpeg_quality = int(np.clip(self.jpeg_quality, 1, 100))
    self.sharpen = float(np.clip(self.sharpen, 0., 1.))


@dataclass 
class PublisherSettings:
  image : ImageSettings
  camera : CameraSettings