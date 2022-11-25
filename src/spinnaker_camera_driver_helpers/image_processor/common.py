from enum import Enum
import cv2

class EncoderError(RuntimeError):
  def __init__(self, msg):
    super(EncoderError, self).__init__(msg)


class ImageEncoding(Enum):
  Bayer_BGGR8 = "bayer_bggr8"
  Bayer_RGGB8 = "bayer_rggb8"
  Bayer_GBRG8 = "bayer_gbrg8"
  Bayer_GRBG8 = "bayer_grbg8"



camera_encodings = dict(
    BayerRG8 = ImageEncoding.Bayer_RGGB8,
    BayerBG8 = ImageEncoding.Bayer_BGGR8,
    BayerGR8 = ImageEncoding.Bayer_GRBG8,
    BayerGB8 = ImageEncoding.Bayer_GBRG8
)


def cv_bayer_bgr(encoding):

  if encoding == ImageEncoding.Bayer_RGGB8:
    return cv2.COLOR_BAYER_BG2BGR
  elif encoding == ImageEncoding.Bayer_BGGR8:
    return cv2.COLOR_BAYER_RG2BGR
  elif encoding == ImageEncoding.Bayer_GRBG8:
    return cv2.COLOR_BAYER_GB2BGR
  elif encoding == ImageEncoding.Bayer_GBRG8:
    return cv2.COLOR_BAYER_GR2BGR  
  else:
    raise ValueError(f"Encoding not implemented {encoding}")


def cv_bayer_bgra(encoding):

  if encoding == ImageEncoding.Bayer_RGGB8:
    return cv2.COLOR_BAYER_BG2BGRA
  elif encoding == ImageEncoding.Bayer_BGGR8:
    return cv2.COLOR_BAYER_RG2BGRA
  elif encoding == ImageEncoding.Bayer_GRBG8:
    return cv2.COLOR_BAYER_GB2BGRA
  elif encoding == ImageEncoding.Bayer_GBRG8:
    return cv2.COLOR_BAYER_GR2BGRA
  else:
    raise ValueError(f"Encoding not implemented {encoding}")
