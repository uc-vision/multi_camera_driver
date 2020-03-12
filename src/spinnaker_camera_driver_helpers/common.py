import tf2_ros
import tf_conversions
import yaml

import rospy

from cv_bridge import CvBridge
from std_msgs.msg import Header
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo

from camera_geometry.camera import Camera, Transform

from camera_geometry_ros.conversions import camera_info_msg, transform_msg
from camera_geometry.import_calibration import import_cameras
from camera_geometry import image_utils, json, util

from tf.transformations import quaternion_from_matrix



def load_config(config_file):
    if config_file is not None:
        with open(config_file) as config_file:
            return yaml.load(config_file, Loader=yaml.Loader)    
    else:
        return None


def load_calibration(calibration_file):
    if calibration_file is not None:
        calib_data = json.load_json(calibration_file)
        return import_cameras(calib_data)
    else:
        return {}, {}


class ImagePublisher(object):
    def __init__(self, name, camera_info, queue_size=4):
        super(ImagePublisher, self).__init__()

        self.bridge = CvBridge()
        self.name = name

        self.image_publisher = rospy.Publisher("{}/image_raw".format(self.name), Image, queue_size=queue_size)
        self.info_publisher = rospy.Publisher("{}/camera_info".format(self.name), CameraInfo, queue_size=queue_size)

        self.cam_info = camera_info

    def publish(self, image, timestamp, encoding="rgb8"):

        header = Header()
        header.stamp = timestamp
        header.frame_id = self.name

        image_msg = self.bridge.cv2_to_imgmsg(image, encoding=encoding)
        image_msg.header = header
        
        self.cam_info.header = header

        self.info_publisher.publish(self.cam_info)
        self.image_publisher.publish(image_msg)



def publish_extrinsics(extrinsics):

    stamp = rospy.Time.now()
    broadcaster = tf2_ros.StaticTransformBroadcaster()

    for child_id, transform in extrinsics.items():       
        msg = transform_msg(transform, child_id, stamp)
        broadcaster.sendTransform(msg)
    return broadcaster