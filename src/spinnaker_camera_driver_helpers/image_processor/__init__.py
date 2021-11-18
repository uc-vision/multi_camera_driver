from .common import EncoderError


def image_backend(name):
  if name == 'torch_nvjpeg':
    from . import torch_nvjpeg
    return torch_nvjpeg.Processor
  elif name == 'turbo_jpeg':
    from . import turbo_jpeg
    return turbo_jpeg.Processor
  else:
    raise RuntimeError(f"Unknown image backend {name} options are: turbo_jpeg | torch_nvjpeg")  


  