
from ..publisher import ImageSettings
from cached_property import cached_property

import cv2
from nvjpeg_torch import Jpeg, JpegException
import torch

from .common import EncoderError


class Processor(object):
  def __init__(self, settings : ImageSettings):
    self.encoder = Jpeg()
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
    def color(self):

      if self.settings.encoding == "bayer_bggr8":
        return cv2.cvtColor(self.raw, cv2.COLOR_BAYER_RG2BGR)
      elif self.settings.encoding == "bayer_rggb8":
        return cv2.cvtColor(self.raw, cv2.COLOR_BAYER_BG2BGR)
      else:
        raise RuntimeException(f"bayer encoding not implemented {self.settings.encoding}")

    def encode(self, image):
      with torch.inference_mode():
        try:
          channels_last = torch.from_numpy(image).cuda()

          return self.parent.encoder.encode(
              channels_last, quality=self.settings.jpeg_quality).numpy().tobytes()
        except JpegException as e:
          raise EncoderError(str(e))

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

