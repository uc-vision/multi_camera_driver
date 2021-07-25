

from camera_geometry_ros.lazy_publisher import LazyPublisher
from camera_geometry_ros.conversions import camera_info_msg

from sensor_msgs.msg import CameraInfo, CompressedImage, Image
from std_msgs.msg import Header

from nvjpeg_torch import Jpeg
import torch
from debayer import Debayer3x3

import torch.nn.functional as F

from cv_bridge import CvBridge
from dataclasses import dataclass
from cached_property import cached_property



@dataclass
class ImageSettings:
  preview_size : int = 400
  encoding : str = 'bayer_bggr8' 
  device : str = 'cuda:0'
  queue_size : int = 4
  quality : int = 90




class CameraOutputs(object):
    def __init__(self, parent, image_raw):
        self.parent = parent
        self.image_raw = image_raw


    @property 
    def settings(self) -> ImageSettings:
      return self.parent.settings

    @cached_property
    def cuda_rgb(self):
      bayer = torch.from_numpy(self.image_raw).cuda()

      batched = bayer.view(1, 1, *bayer.shape)
      rgb = self.parent.debayer(batched.to(dtype=torch.float16))
      return rgb.to(dtype=torch.uint8).flip(1)


    @cached_property
    def image_color(self):
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

    @cached_property
    def camera_info(self):
      if self.parent.calibration is not None:
        calibration = self.parent.calibration.resize_image(
                (self.image_raw.shape[1], self.image_raw.shape[0]))
        return camera_info_msg(calibration)
      else:
        return CameraInfo()


class CameraPublisher():
  def __init__(self, camera_name, settings, calibration=None):
    
    self.camera_name = camera_name
    self.settings = settings

    self.encoder = Jpeg()
    self.debayer = Debayer3x3().to(dtype=torch.float16, device=self.settings.device)
    self.calibration = calibration
    
    bridge = CvBridge()

    topics = {
        "image_raw"        : (Image, lambda data: bridge.cv2_to_imgmsg(data.image_raw, encoding=settings.encoding)),
        "image_color"      : (Image, lambda data: bridge.cv2_to_imgmsg(data.image_color, encoding="bgr8")),
        "compressed"       : (CompressedImage, lambda data: CompressedImage(data = data.compressed, format = "jpeg")), 
        "preview/compressed" :  (CompressedImage, lambda data: CompressedImage(data = data.preview, format = "jpeg")),
        "camera_info" : (CameraInfo, lambda data: data.camera_info)
    }

    self.publisher = LazyPublisher(topics, self.register, name=self.camera_name)

  def register(self):
    return []     # Here's where the lazy subscriber subscribes to it's inputs (none for this)


  def update_calibration(self, camera):
    self.calibration = camera

  def set_option(self, option, value):
    if option == "preview_size":
      self.settings.preview_size = value
    elif option == "quality":
      assert value > 0 and value <= 100
      self.settings.quality = value
    else:
      assert False, f"unknown option {option}"


  def publish(self, image_data, timestamp, seq):
    return self.publisher.publish(
      data = CameraOutputs(self, image_data), 
      header = Header(frame_id=self.camera_name, stamp=timestamp, seq=seq)
    )


  def stop(self):
    pass

  def start(self):
    pass



    
    


