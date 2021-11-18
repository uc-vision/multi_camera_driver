
from ..publisher import ImageSettings
from cached_property import cached_property

import cv2
from turbojpeg import TurboJPEG

from .common import EncoderError


class Processor(object):
  def __init__(self, settings : ImageSettings):
    self.encoder = TurboJPEG()
    self.settings = settings


  def __call__(self, raw):
    return ImageOutputs(self, raw)


class ImageOutputs(object):
    def __init__(self, parent, raw):
        self.parent = parent
        self.raw = raw

    @property 
    def settings(self) -> ImageSettings:
      return self.parent.settings

    @cached_property
    def image_color(self):
      return cv2.cvtColor(self.raw, cv2.COLOR_BAYER_BG2BGR)

    def encode(self, image):
      return self.parent.encoder.encode(image, quality=self.settings.quality)

    @cached_property 
    def compressed(self):
      return self.encode(self.image_color)

    @cached_property 
    def preview(self):
      img_h, img_w, _ = self.image_color.shape

      w = self.settings.preview_size
      h = int(img_h * (w / img_w)) 

      preview_rgb = cv2.resize(self.image_color, dsize=(w, h))
      return self.encode(preview_rgb)

