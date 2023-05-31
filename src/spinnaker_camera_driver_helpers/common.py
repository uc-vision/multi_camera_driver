from dataclasses import dataclass, field
from datetime import datetime
from enum import Enum
from traceback import format_exc
from typing import Any, Dict, Optional, Tuple
from beartype import beartype
import cv2
import PySpin
import numpy as np
import torch
import rospy

import camera_geometry 

class EncoderError(RuntimeError):
  def __init__(self, msg):
    super(EncoderError, self).__init__(msg)

class BayerPattern(Enum):
  BGGR = "bggr"
  RGGB = "rggb"
  GBRG = "gbrg"
  GRBG = "grbg"


class ImageEncoding(Enum):
  Bayer_BGGR8 = "bayer_bggr8"
  Bayer_RGGB8 = "bayer_rggb8"
  Bayer_GBRG8 = "bayer_gbrg8"
  Bayer_GRBG8 = "bayer_grbg8"
  Bayer_BGGR12 = "bayer_bggr12"
  Bayer_RGGB12 = "bayer_rggb12"
  Bayer_GBRG12 = "bayer_gbrg12"
  Bayer_GRBG12 = "bayer_grbg12"

  Bayer_BGGR16 = "bayer_bggr16"
  Bayer_RGGB16 = "bayer_rggb16"
  Bayer_GBRG16 = "bayer_gbrg16"
  Bayer_GRBG16 = "bayer_grbg16"


pyspin_encoding = {
  PySpin.PixelFormat_BayerRG8: ImageEncoding.Bayer_RGGB8,
  PySpin.PixelFormat_BayerRG12p: ImageEncoding.Bayer_RGGB12,
  PySpin.PixelFormat_BayerRG16: ImageEncoding.Bayer_RGGB16,
  PySpin.PixelFormat_BayerGR8: ImageEncoding.Bayer_GRBG8,
  PySpin.PixelFormat_BayerGR12p: ImageEncoding.Bayer_GRBG12,
  PySpin.PixelFormat_BayerGR16: ImageEncoding.Bayer_GRBG16,
  PySpin.PixelFormat_BayerGB8: ImageEncoding.Bayer_GBRG8,
  PySpin.PixelFormat_BayerGB12p: ImageEncoding.Bayer_GBRG12,
  PySpin.PixelFormat_BayerGB16: ImageEncoding.Bayer_GBRG16,
  PySpin.PixelFormat_BayerBG8: ImageEncoding.Bayer_BGGR8,
  PySpin.PixelFormat_BayerBG12p: ImageEncoding.Bayer_BGGR12,
  PySpin.PixelFormat_BayerBG16: ImageEncoding.Bayer_BGGR16,
}


def from_pyspin(encoding):
  if encoding in pyspin_encoding:
    return pyspin_encoding[encoding]
  else:
    raise ValueError(f"Encoding not implemented {encoding}")



def encoding_bits(encoding):
  if encoding in [ImageEncoding.Bayer_BGGR8, ImageEncoding.Bayer_RGGB8, ImageEncoding.Bayer_GBRG8, ImageEncoding.Bayer_GRBG8]:
    return 8
  elif encoding in [ImageEncoding.Bayer_BGGR12, ImageEncoding.Bayer_RGGB12, ImageEncoding.Bayer_GBRG12, ImageEncoding.Bayer_GRBG12]:
    return 12
  elif encoding in [ImageEncoding.Bayer_BGGR16, ImageEncoding.Bayer_RGGB16, ImageEncoding.Bayer_GBRG16, ImageEncoding.Bayer_GRBG16]:
    return 16
  else:
    raise ValueError(f"Encoding not implemented {encoding}")


def bayer_pattern(encoding):
  if encoding in [ImageEncoding.Bayer_BGGR8, ImageEncoding.Bayer_BGGR12, ImageEncoding.Bayer_BGGR16]:
    return BayerPattern.BGGR
  elif encoding in [ImageEncoding.Bayer_RGGB8, ImageEncoding.Bayer_RGGB12, ImageEncoding.Bayer_RGGB16]:
    return BayerPattern.RGGB
  elif encoding in [ImageEncoding.Bayer_GBRG8, ImageEncoding.Bayer_GBRG12, ImageEncoding.Bayer_GBRG16]:
    return BayerPattern.GBRG
  elif encoding in [ImageEncoding.Bayer_GRBG8, ImageEncoding.Bayer_GRBG12, ImageEncoding.Bayer_GRBG16]:
    return BayerPattern.GRBG
  else:
    raise ValueError(f"Encoding not implemented {encoding}")
  


camera_encodings = dict(
    BayerRG8 = ImageEncoding.Bayer_RGGB8,
    BayerBG8 = ImageEncoding.Bayer_BGGR8,
    BayerGR8 = ImageEncoding.Bayer_GRBG8,
    BayerGB8 = ImageEncoding.Bayer_GBRG8,
    BayerRG12p = ImageEncoding.Bayer_RGGB12,
    BayerBG12p = ImageEncoding.Bayer_BGGR12,
    BayerGR12p = ImageEncoding.Bayer_GRBG12,
    BayerGB12p = ImageEncoding.Bayer_GBRG12,
    BayerRG16 = ImageEncoding.Bayer_RGGB16,
    BayerBG16 = ImageEncoding.Bayer_BGGR16,
    BayerGR16 = ImageEncoding.Bayer_GRBG16,
    BayerGB16 = ImageEncoding.Bayer_GBRG16,
)


def cv_bayer_bgr(encoding):

  if encoding in [ImageEncoding.Bayer_RGGB8, ImageEncoding.Bayer_RGGB16]:
    return cv2.COLOR_BAYER_BG2BGR
  elif encoding in [ImageEncoding.Bayer_BGGR8, ImageEncoding.Bayer_BGGR16]:
    return cv2.COLOR_BAYER_RG2BGR
  elif encoding in [ImageEncoding.Bayer_GRBG8, ImageEncoding.Bayer_GRBG16]:
    return cv2.COLOR_BAYER_GB2BGR
  elif encoding in [ImageEncoding.Bayer_GBRG8, ImageEncoding.Bayer_GBRG16]:
    return cv2.COLOR_BAYER_GR2BGR
  else:
    raise ValueError(f"Encoding not implemented {encoding}")


def cv_bayer_bgra(encoding):

  if encoding in [ImageEncoding.Bayer_RGGB8, ImageEncoding.Bayer_RGGB16]:
    return cv2.COLOR_BAYER_BG2BGRA
  elif encoding in [ImageEncoding.Bayer_BGGR8, ImageEncoding.Bayer_BGGR16]:
    return cv2.COLOR_BAYER_RG2BGRA
  elif encoding in [ImageEncoding.Bayer_GRBG8, ImageEncoding.Bayer_GRBG16]:
    return cv2.COLOR_BAYER_GB2BGRA
  elif encoding in [ImageEncoding.Bayer_GBRG8, ImageEncoding.Bayer_GBRG16]:
    return cv2.COLOR_BAYER_GR2BGRA
  else:
    raise ValueError(f"Encoding not implemented {encoding}")



class IncompleteImageError(Exception):
  def __init__(self, status):
    self.status = status
    
  def __str__(self):
    return f"Incomplete image: {self.status}"

@beartype
@dataclass
class CameraImage:
  camera_name: str
  image_data: torch.Tensor
  timestamp: rospy.Time
  seq: int
  image_size: Tuple[int, int]
  encoding: ImageEncoding

  def __repr__(self):
    date = datetime.fromtimestamp(self.timestamp.to_sec())
    pretty_time = date.strftime("%H:%M:%S.%f")
    w, h = self.image_size

    return f"CameraImage({self.camera_name}, {w}x{h}, {self.image_data.shape[0]}:{str(self.image_data.dtype)}, {self.encoding.value}, {pretty_time}, seq={self.seq})"

  
@beartype
@dataclass
class CameraSettings:
  name : str
  master_id:Optional[str]

  connection_speed:str
  time_offset_sec:rospy.Duration
  serial:str
  
  framerate : Optional[float]

  image_size:Tuple[int, int]
  encoding : ImageEncoding

  calibration: Optional[camera_geometry.Camera] = None
  settings: Dict[str, Any] = field(default_factory=dict)

  @property
  def is_master(self):
    return self.name == self.master_id 