from functools import partial
from pathlib import Path

import rospy
import torch
import torch.nn.functional as F
from cached_property import cached_property
from nvjpeg_torch import Jpeg, JpegException

from spinnaker_camera_driver_helpers.image_settings import PublisherSettings

from .common import EncoderError, cv_conversion
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

   

    @cached_property
    def cuda_raw(self):
      raw_gpu = cv2.cuda_GpuMat()
      raw_gpu.upload(self.raw)
      return raw_gpu


    @property  
    def settings(self) -> PublisherSettings:
      return self.parent.settings
    
    @cached_property
    def cuda_rgb(self):
      bgr = cv2.cuda.cvtColor(self.cuda_raw, 
        cv_conversion(self.settings.camera.encoding))

      if self.settings.image.resize_width:
        w, h = bgr.size()
        scale_factor = self.settings.image.resize_width / w
        bgr = cv2.cuda.resize(bgr, (self.settings.image.resize_width, int(h * scale_factor)))

      return bgr


    def encode(self, image):
      with torch.inference_mode():
        try:
          image = torch.asarray(CudaArrayInterface(image), device=self.device).contiguous()
          return self.parent.jpeg.encode(
              image, 
              quality=self.settings.image.jpeg_quality, 
              input_format = Jpeg.BGRI).numpy().tobytes()

        except JpegException as e:
          raise EncoderError(str(e))

    @cached_property 
    def compressed(self):
      return self.encode(self.cuda_rgb)


    @cached_property 
    def preview(self):
      img_w, img_h = self.cuda_rgb.size()

      w = self.settings.image.preview_size
      h = int(img_h * (w / img_w)) 

      preview_rgb = cv2.resize(self.cuda_rgb, dsize=(w, h))
      return self.encode(preview_rgb)