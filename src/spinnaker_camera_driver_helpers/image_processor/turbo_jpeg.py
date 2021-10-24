
from ..publisher import ImageSettings
from cached_property import cached_property

import cv2
from turbojpeg import TurboJPEG

class Processor(object):
  def __init__(self, settings : ImageSettings):
    self.encoder = TurboJPEG()
    self.settings = settings


  def __call__(self, raw):
    return ImageOutputs(self, raw)


class ImageOutputs(object):
    def __init__(self, parent, raw):
        self.parent = parent
        self.raw = cv2.UMat(raw)

    @property 
    def settings(self) -> ImageSettings:
      return self.parent.settings

    @cached_property
    def image_color(self):
      return cv2.cvtColor(self.raw, cv2.COLOR_BAYER_BG2RGB)

    def encode(self, image):
      return self.parent.encoder.encode(image.get(), quality=self.settings.quality)

    @cached_property 
    def compressed(self):
      return self.encode(self.image_color)

    @cached_property 
    def preview(self):
      preview_rgb = cv2.resize(self.image_color, size=self.settings.preview_size)
      return self.encode(preview_rgb)

