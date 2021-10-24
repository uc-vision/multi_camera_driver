from nvjpeg_torch import Jpeg
import torch
from debayer import Debayer3x3

import torch.nn.functional as F
from ..publisher import ImageSettings

from cached_property import cached_property


class Processor(object):
  def __init__(self, settings : ImageSettings):
    self.encoder = Jpeg()
    self.debayer = Debayer3x3().to(dtype=torch.float16, device=self.settings.device)
    self.settings = settings

  def __call__(self, image_raw):
    return ImageOutputs(self, image_raw)


class ImageOutputs(object):
    def __init__(self, parent, image_raw):
        self.parent = parent
        self.raw = image_raw


    @property 
    def settings(self) -> ImageSettings:
      return self.parent.settings

    @cached_property
    def cuda_rgb(self):
      bayer = torch.from_numpy(self.raw).cuda()

      batched = bayer.view(1, 1, *bayer.shape)
      rgb = self.parent.debayer(batched.to(dtype=torch.float16))
      return rgb.to(dtype=torch.uint8).flip(1)


    @cached_property
    def color(self):
      image = self.cuda_rgb.permute(0, 2, 3, 1).squeeze(0).cpu().numpy()
      return image

    def encode(self, image):
      channels_last = image.permute(0, 2, 3, 1).squeeze(0).contiguous()

      return self.parent.encoder.encode(
          channels_last, quality=self.settings.quality).numpy().tobytes()

    @cached_property 
    def compressed(self):
      return self.encode(self.cuda_rgb)


    @cached_property 
    def preview(self):
      preview_rgb = F.interpolate(self.cuda_rgb, size=self.settings.preview_size)
      return self.encode(preview_rgb)