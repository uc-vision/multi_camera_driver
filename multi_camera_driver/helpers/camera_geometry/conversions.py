from dataclasses import dataclass
import numpy as np
from camera_geometry import camera_models
from camera_geometry.transforms import expand_identity
from geometry_msgs.msg import TransformStamped, Transform
from sensor_msgs.msg import CameraInfo


import ros2_numpy
import rospy2 as rospy



def camera_info_msg(camera:camera_models.Camera):
  cam_info = CameraInfo()
  width, height = camera.image_size
  cam_info.width = int(width)
  cam_info.height = int(height)

  cam_info.distortion_model = str('plumb_bob')

  cam_info.K = camera.intrinsic.flatten().tolist()
  cam_info.D = camera.dist.flatten().tolist()

  cam_info.P = expand_identity(camera.intrinsic,
                               shape=(3, 4)).flatten().tolist()
  return cam_info


def empty_camera_info_msg(width:int, height:int):
  cam_info = CameraInfo()
  cam_info.width = int(width)
  cam_info.height = int(height)

  cam_info.distortion_model = str('plumb_bob')

  cam_info.K = np.zeros(3).flatten().tolist()
  cam_info.D = np.zeros(5).flatten().tolist()

  cam_info.P = np.zeros(3, 4).flatten().tolist()
  return cam_info


def camera_from_msg(camera_info:CameraInfo, parent_t_camera=np.eye(4)):

  return camera_models.Camera(image_size=(camera_info.width,
                                          camera_info.height),
                              dist=np.array(camera_info.D),
                              intrinsic=np.array(camera_info.K).reshape((3, 3)),
                              parent_t_camera=parent_t_camera)


def rectified_info_msg(rectified:camera_models.RectifiedCamera):
  cam_info = camera_info_msg(rectified.camera)
  cam_info.R = rectified.rotation.flatten().tolist()
  cam_info.P = rectified.intrinsic.flatten().tolist()

  x, y, w, h = rectified.roi
  roi = cam_info.roi
  roi.x_offset = x
  roi.y_offset = y
  roi.width = w
  roi.height = h
  return cam_info


def rectified_from_msg(cam_info:CameraInfo, depth_t_disparity:np.ndarray, parent_t_camera=np.eye(4)):
  camera = camera_from_msg(cam_info, parent_t_camera)

  rotation = np.array(cam_info.R).reshape(3, 3)
  intrinsic = np.array(cam_info.P).reshape(3, 4)

  roi = cam_info.roi
  roi = (roi.x_offset, roi.y_offset, roi.width, roi.height)

  return camera_models.RectifiedCamera(camera,
                                       rotation,
                                       intrinsic,
                                       depth_t_disparity,
                                       roi=roi)


@dataclass 
class RelativeTransform:
  transform: np.ndarray
  parent: str
  child: str



def transform_from_msg(msg:TransformStamped):
  return RelativeTransform(transform=ros2_numpy.numpify(msg.transform),
                parent=msg.header.frame_id,
                child=msg.child_frame_id)



def transform_stamped(transform:np.ndarray, parent:str, child:str, stamp:rospy.Time) -> TransformStamped:
  return TransformStamped(
    transform=ros2_numpy.msgify(Transform, transform),
    header=rospy.Header(frame_id=parent, stamp=stamp),
    child_frame_id=child
  )
