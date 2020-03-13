import tf2_ros
import tf_conversions
import yaml

import rospy
import message_filters

from cv_bridge import CvBridge
from std_msgs.msg import Header
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo
from maara_msgs.msg import StereoCameraInfo


from camera_geometry.camera import Camera, Transform

import camera_geometry_ros.conversions as conversions
from camera_geometry.import_calibration import import_cameras
from camera_geometry import image_utils, json, util, stereo_pair



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




def stereo_info_msg(pair):
    assert isinstance(pair, stereo_pair.StereoPair)

    left, right = pair.rectified

    msg = StereoCameraInfo()
    msg.left_info = conversions.rectified_info_msg(left)
    msg.right_info = conversions.rectified_info_msg(right)

    msg.T_left_right = pair.translation.flatten().tolist()
    msg.R_left_right = pair.rotation.flatten().tolist()

    msg.Q = left.disparity_to_depth.flatten().tolist()
    return msg
  
def defer(f, args):
    thread = Thread(target=f, args=args)
    thread.start()
    return thread

class StereoPublisher(object):

    def __init__(self, name, left, right, queue_size=4):

        self.buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.buffer)

        self.frames = (left, right)
        self.name = name

        topics = ["{}/camera_info".format(camera) for camera in [left, right]]

        self.publisher = rospy.Publisher("{}/stereo_info".format(self.name), StereoCameraInfo, queue_size=queue_size)

        self.info_publishers = [
            rospy.Publisher("{}/{}/camera_info".format(self.name, camera), CameraInfo, queue_size=queue_size)
                for camera in ['left', 'right']
        ]

        subscribers = [message_filters.Subscriber(topic, CameraInfo) for topic in topics]

        sync = message_filters.TimeSynchronizer(subscribers, queue_size=queue_size)
        sync.registerCallback(self.frame_callback)


    def frame_callback(self, left_info, right_info):
        try:
            msg = self.buffer.lookup_transform(self.frames[1], self.frames[0], rospy.Time())
            _, transform = conversions.transform_from_msg(msg)
            
            
            left = conversions.camera_from_msg(left_info)
            right = conversions.camera_from_msg(right_info, extrinsic = transform.extrinsic)

            pair = stereo_pair.rectify_pair(left, right)
                     
            stereo_info = stereo_info_msg(pair)
            header = stereo_info.header
            header.stamp = left_info.header.stamp
            header.frame_id = self.name

            for msg in [stereo_info, stereo_info.left_info, stereo_info.right_info]:
                msg.header = header

            self.publisher.publish(stereo_info)
            
            for publisher, msg in zip(self.info_publishers, [stereo_info.left_info, stereo_info.right_info]):
                publisher.publish(msg)

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            pass    


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
        msg = conversions.transform_msg(transform, child_id, stamp)
        broadcaster.sendTransform(msg)
    return broadcaster