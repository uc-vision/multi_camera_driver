from nvjpeg_torch import Jpeg, JpegException
import torch
from debayer import DebayerSplit, Layout

from .common import EncoderError

import torch.nn.functional as F
from ..publisher import ImageSettings

from cached_property import cached_property


class Processor(object):
  def __init__(self, settings : ImageSettings):
    self.settings = settings

    self.encoder = Jpeg()
    layouts = dict(
      bayer_rggb8 = Layout.RGGB, 
      bayer_grbg8 = Layout.GRBG,
      bayer_gbrg8 = Layout.GBRG,
      bayer_bggr8 = Layout.BGGR
    )

    assert settings.encoding in layouts

    self.dtype = torch.float16
    self.debayer = DebayerSplit(layouts[settings.encoding]
      ).to(dtype=self.dtype, device=self.settings.device, 
          memory_format=torch.channels_last)

  def __call__(self, image_raw):
    return ImageOutputs(self, image_raw, self.dtype, self.settings.device)


class ImageOutputs(object):
    def __init__(self, parent, image_raw, dtype, device):
        self.parent = parent
        self.image_raw = image_raw
        self.dtype = dtype
        self.device = device

    @cached_property
    def raw(self):
      return torch.from_numpy(self.image_raw).to(device=self.device)

    @property 
    def settings(self) -> ImageSettings:
      return self.parent.settings

    @cached_property
    def cuda_rgb(self):
      with torch.inference_mode():
        bayer = self.raw

        batched = bayer.view(1, 1, *bayer.shape).to(memory_format=torch.channels_last)

        rgb = self.parent.debayer(batched.to(dtype=self.dtype))
        return rgb.to(dtype=torch.uint8).flip(1)


    @cached_property
    def color(self):
      with torch.inference_mode():
        image = self.cuda_rgb.permute(0, 2, 3, 1).squeeze(0).cpu().numpy()
        return image

    def encode(self, image):
      with torch.inference_mode():
        try:
          channels_last = image.squeeze(0)

          return self.parent.encoder.encode(
              channels_last, quality=self.settings.quality).numpy().tobytes()
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
