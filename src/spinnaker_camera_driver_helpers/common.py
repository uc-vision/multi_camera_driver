import tf2_ros
import tf_conversions
import yaml

import rospy
import message_filters

from cv_bridge import CvBridge
import cv2

from std_msgs.msg import Header
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo
from maara_msgs.msg import StereoCameraInfo

from camera_geometry.camera import Camera, Transform

import camera_geometry_ros.conversions as conversions
from camera_geometry.import_calibration import import_cameras
from camera_geometry import image_utils, json, util, stereo_pair

from camera_geometry_ros.conversions import camera_info_msg
from camera_geometry_ros.stereo_pair import stereo_info_msg


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




  
def defer(f, args):
    thread = Thread(target=f, args=args)
    thread.start()
    return thread


def equal_pair(pair, left, right):
    return pair.cameras[0].approx_eq(left) and pair.cameras[1].approx_eq(right)

class StereoPublisher(object):
    def __init__(self, name, left, right, resize=None, queue_size=1):

        self.buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.buffer)

        self.frames = (left, right)
        self.name = name

        self.pair = None
        self.bridge = CvBridge()

        self.resize = resize

        info_topics = ["{}/camera_info".format(camera) for camera in [left, right]]
        image_topics = ["{}/image_raw".format(camera) for camera in [left, right]]

        self.stereo_publisher = rospy.Publisher("{}/stereo_info".format(self.name), 
            StereoCameraInfo, queue_size=queue_size)

        self.image_publishers = [ImagePublisher("{}/{}".format(self.name, camera), encoding="bgr8",
            image_topic="image_color_rect", queue_size=queue_size) for camera in ["left", "right"]]

        image_subscribers = []

        image_subscribers = [message_filters.Subscriber(topic, Image, queue_size=queue_size, buff_size=2**24) for topic in image_topics]        
        info_subscribers = [message_filters.Subscriber(topic, CameraInfo, queue_size=queue_size) for topic in info_topics]   
               
        sync = message_filters.TimeSynchronizer(info_subscribers + image_subscribers, queue_size=1)
        sync.registerCallback(self.frame_callback)


    def decode_rectify(self, image_msg, calibration):
        image = self.bridge.imgmsg_to_cv2(image_msg, desired_encoding="bgr8")
        source_size = (image.shape[1], image.shape[0])

        if source_size != self.pair.image_size:
            image = cv2.resize(image, self.pair.image_size, interpolation=cv2.INTER_AREA)
        
        return calibration.rectify(image)


    def frame_callback(self, left_info, right_info, left_msg, right_msg):
        try:
            msg = self.buffer.lookup_transform(self.frames[0], self.frames[1], rospy.Time())
            _, transform = conversions.transform_from_msg(msg)
                    
            left = conversions.camera_from_msg(left_info)
            right = conversions.camera_from_msg(right_info, extrinsic = transform.extrinsic)

            if self.resize is not None:
                left = left.resize_image(self.resize)
                right = right.resize_image(self.resize)

            if self.pair is None or (not equal_pair(self.pair, left, right)):
                self.pair = stereo_pair.rectify_pair(left, right)
                     
            timestamp = left_info.header.stamp

            stereo_info = stereo_info_msg(self.pair)
            stereo_info.header = make_header(self.name,  timestamp)
            self.stereo_publisher.publish(stereo_info)

            for publisher, image_msg, calibration in zip(self.image_publishers, [left_msg, right_msg], self.pair.rectified):
                image = self.decode_rectify(image_msg, calibration)

                cam_info = conversions.rectified_info_msg(calibration)
                publisher.publish(image, timestamp, cam_info)
               
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            pass    

def make_header(frame_id, timestamp, seq=0):
    header = Header()
    header.stamp = timestamp
    header.frame_id = frame_id
    header.seq = seq
    return header


class ImagePublisher(object):
    def __init__(self, name,  image_topic="image_raw", encoding="passthrough", queue_size=4):
        super(ImagePublisher, self).__init__()

        self.bridge = CvBridge()
        self.name = name

        self.encoding = encoding

        self.image_publisher = rospy.Publisher("{}/{}".format(self.name, image_topic), Image, queue_size=queue_size)
        self.info_publisher = rospy.Publisher("{}/camera_info".format(self.name), CameraInfo, queue_size=queue_size)

        self.seq = 0

    def publish(self, image, timestamp, cam_info=None):
        header = make_header(self.name, timestamp, self.seq + 1)

        cam_info = cam_info or CameraInfo()        
        cam_info.header = header

        image_msg = self.bridge.cv2_to_imgmsg(image, encoding=self.encoding)
        image_msg.header = header

        self.info_publisher.publish(cam_info)
        self.image_publisher.publish(image_msg)

        self.seq += 1


class CalibratedPublisher(object):
    def __init__(self, name, calibration=None, **kwargs):
        self.publisher = ImagePublisher(name, **kwargs)
        self.calibration = calibration
    
    def publish(self, image, timestamp):
        cam_info = CameraInfo()        
        if self.calibration is not None:
            # Adjust calibration to compensate for binning
            calibration = self.calibration.resize_image((image.shape[1], image.shape[0]))
            cam_info = camera_info_msg(calibration)

        self.publisher.publish(image, timestamp, cam_info)


def publish_extrinsics(extrinsics):

    stamp = rospy.Time.now()
    broadcaster = tf2_ros.StaticTransformBroadcaster()

    for child_id, transform in extrinsics.items():      
        msg = conversions.transform_msg(transform, child_id, stamp)
        broadcaster.sendTransform(msg)
    return broadcaster