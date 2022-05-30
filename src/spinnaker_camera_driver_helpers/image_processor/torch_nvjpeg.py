from nvjpeg_torch import Jpeg, JpegException
import torch
from debayer import Debayer5x5

from .common import EncoderError

import torch.nn.functional as F
from ..publisher import ImageSettings

from cached_property import cached_property


class Processor(object):
  def __init__(self, settings : ImageSettings):
    self.settings = settings

    self.encoder = Jpeg()
    self.debayer = Debayer5x5().to(dtype=torch.float16, device=self.settings.device)

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
      with torch.inference_mode():
        bayer = torch.from_numpy(self.raw).cuda()
        batched = bayer.view(1, 1, *bayer.shape)
        rgb = self.parent.debayer(batched.to(dtype=torch.float16))
        return rgb.to(dtype=torch.uint8).flip(1)


    @cached_property
    def color(self):
      with torch.inference_mode():
        image = self.cuda_rgb.permute(0, 2, 3, 1).squeeze(0).cpu().numpy()
        return image

    def encode(self, image):
      with torch.inference_mode():
        try:
          channels_last = image.permute(0, 2, 3, 1).squeeze(0).contiguous()

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
