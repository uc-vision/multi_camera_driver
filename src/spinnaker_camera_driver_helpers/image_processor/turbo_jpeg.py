
from ..publisher import ImageSettings
from cached_property import cached_property

import cv2
from turbojpeg import TurboJPEG

from .common import EncoderError

def cv_conversion(settings):
  if settings.encoding == "bayer_bggr8":
    return cv2.COLOR_BAYER_RG2BGR
  elif settings.encoding == "bayer_rggb8":
    return cv2.COLOR_BAYER_BG2BGR
  else:
    raise RuntimeException(f"bayer encoding not implemented {settings.encoding}")



class Processor(object):
  def __init__(self, settings : ImageSettings):
    self.encoder = TurboJPEG()
    self.settings = settings

    self.conversion = cv_conversion(settings)


  def __call__(self, raw):
    return ImageOutputs(self, raw, self.conversion)


class ImageOutputs(object):
    def __init__(self, parent, raw, conversion):
        self.parent = parent
        self.raw = raw
        self.conversion = conversion

    @property 
    def settings(self) -> ImageSettings:
      return self.parent.settings

    @cached_property
    def color(self):
        return cv2.cvtColor(self.raw, self.conversion)

    def encode(self, image):
      return self.parent.encoder.encode(image, quality=self.settings.jpeg_quality)

    @cached_property 
    def compressed(self):
      return self.encode(self.color)

    @cached_property 
    def preview(self):
      img_h, img_w, _ = self.color.shape

      w = self.settings.preview_size
      h = int(img_h * (w / img_w)) 

      preview_rgb = cv2.resize(self.color, dsize=(w, h))
      return self.encode(preview_rgb)

