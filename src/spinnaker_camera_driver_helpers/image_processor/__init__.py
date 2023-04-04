from .common import EncoderError


def image_backend(name):
  if name == 'torch_nvjpeg':
    from . import torch_nvjpeg
    return torch_nvjpeg.Processor
  else:
    raise RuntimeError(f"Unknown image backend {name} options are: torch_nvjpeg")  


  