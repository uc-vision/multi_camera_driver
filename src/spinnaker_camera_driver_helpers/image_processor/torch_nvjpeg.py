from nvjpeg_torch import Jpeg, JpegException
import torch

from .common import EncoderError

import torch.nn.functional as F
from ..publisher import ImageSettings

from cached_property import cached_property


def debayer_module(settings, dtype=torch.float16):
    from debayer import DebayerSplit, Layout

    layouts = dict(
      bayer_rggb8 = Layout.RGGB, 
      bayer_grbg8 = Layout.GRBG,
      bayer_gbrg8 = Layout.GBRG,
      bayer_bggr8 = Layout.BGGR
    )
    assert settings.encoding in layouts

    debayer = DebayerSplit(layouts[settings.encoding]
      ).to(dtype=dtype, device=settings.device, memory_format=torch.channels_last)

    return debayer

class Processor(object):
  def __init__(self, settings : ImageSettings):
    self.settings = settings

    self.encoder = Jpeg()

    self.dtype = torch.float16
    self.debayer = debayer_module(settings, dtype=self.dtype)

  def __call__(self, raw):
    return ImageOutputs(self, raw, self.dtype, self.settings.device)


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
    def settings(self) -> ImageSettings:
      return self.parent.settings

    @cached_property
    def cuda_rgb(self):
      with torch.inference_mode():
        batched = self.cuda_raw.view(1, 1, *self.cuda_raw.shape
          ).to(memory_format=torch.channels_last)

        rgb = self.parent.debayer(batched.to(dtype=self.dtype))
        return rgb.to(dtype=torch.uint8)

    def encode(self, image):
      with torch.inference_mode():
        try:
          channels_last = image.squeeze(0)

          return self.parent.encoder.encode(
              channels_last, 
              quality=self.settings.quality,
              input_format = Jpeg.RGB).numpy().tobytes()
        except JpegException as e:
          raise EncoderError(str(e))

    @cached_property 
    def compressed(self):
      return self.encode(self.cuda_rgb)


    @cached_property 
    def preview(self):
      with torch.inference_mode():      
        preview_rgb = F.interpolate(self.cuda_rgb, size=self.settings.preview_size)
        return self.encode(preview_rgb)
