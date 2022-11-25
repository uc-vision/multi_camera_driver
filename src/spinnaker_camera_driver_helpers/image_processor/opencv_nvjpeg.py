from functools import partial
from pathlib import Path

import rospy
import torch
import torch.nn.functional as F
from cached_property import cached_property
from nvjpeg_torch import Jpeg, JpegException

from spinnaker_camera_driver_helpers.image_settings import PublisherSettings
from typeguard import typechecked

from .common import EncoderError, cv_bayer_bgr
import cv2

type_map = {
    cv2.CV_8U: "u1", 
    cv2.CV_8S: "i1",
    cv2.CV_16U: "u2", 
    cv2.CV_16S: "i2",
    cv2.CV_32S: "i4",
    cv2.CV_32F: "f4", 
    cv2.CV_64F: "f8",
}


class CudaArrayInterface:
  def __init__(self, gpu_mat):
      w, h = gpu_mat.size()

      self.__cuda_array_interface__ = {
          "version": 2,
          "shape": (h, w, gpu_mat.channels()),
          "data": (gpu_mat.cudaPtr(), False),
          "typestr": type_map[gpu_mat.depth()],
          "strides": (gpu_mat.step, gpu_mat.elemSize(), gpu_mat.elemSize1()),      
        }



class Processor(object):
  @typechecked
  def __init__(self, settings : PublisherSettings):
    self.settings = settings
    self.jpeg = Jpeg()


  def update_settings(self, settings):
    self.settings = settings
    return False


  def __call__(self, raw):
    return ImageOutputs(self, raw, self.settings.image.device)


class ImageOutputs(object):
    def __init__(self, parent, raw, device):
        self.parent = parent
        self.raw = raw
        self.device = device
        self.lap_filter = cv2.cuda.createLaplacianFilter(cv2.CV_8UC4, cv2.CV_8UC4, 3, 1)

   

    @cached_property
    def cuda_raw(self):
      raw_gpu = cv2.cuda_GpuMat()
      raw_gpu.upload(self.raw)
      return raw_gpu


    @property  
    def settings(self) -> PublisherSettings:
      return self.parent.settings
    
    @cached_property
    def cuda_bgra(self):

      bgr = cv2.cuda.cvtColor(self.cuda_raw, 
        cv_bayer_bgr(self.settings.camera.encoding))
      bgra = cv2.cuda.cvtColor(bgr, cv2.COLOR_BGR2BGRA)

        
      if self.settings.image.resize_width:
        w, h = bgra.size()
        scale_factor = self.settings.image.resize_width / w
        bgra = cv2.cuda.resize(bgra, (self.settings.image.resize_width, int(h * scale_factor)), interpolation=cv2.INTER_AREA)

      if self.settings.image.is_sharpening:
        factor = self.settings.image.sharpen

        lap = self.lap_filter.apply(bgra)
        bgra = cv2.cuda.addWeighted(bgra, 1.0, lap, -factor, 0.0)

      return bgra


    def encode(self, bgra):
      with torch.inference_mode():
        try:
          bgr = cv2.cuda.cvtColor(bgra, cv2.COLOR_BGRA2BGR)
          torch_bgr = torch.asarray(CudaArrayInterface(bgr), device=self.device).contiguous()
          return self.parent.jpeg.encode(
              torch_bgr, 
              quality=self.settings.image.jpeg_quality, 
              input_format = Jpeg.BGRI).numpy().tobytes()

        except JpegException as e:
          raise EncoderError(str(e))

    @cached_property 
    def compressed(self):
      return self.encode(self.cuda_bgra)


    @cached_property 
    def preview(self):
      img_w, img_h = self.cuda_bgra.size()

      w = self.settings.image.preview_size
      h = int(img_h * (w / img_w)) 

      preview_rgb = cv2.resize(self.cuda_bgra, dsize=(w, h))
      return self.encode(preview_rgb)