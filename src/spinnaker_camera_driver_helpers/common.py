import tf2_ros
import yaml

import rospy
import message_filters

from functools import partial

import cv2
from cv_bridge import CvBridge

from std_msgs.msg import Header
from sensor_msgs.msg import Image, CompressedImage
from sensor_msgs.msg import CameraInfo
from cares_msgs.msg import StereoCameraInfo

import camera_geometry_ros.conversions as conversions
from camera_geometry.calib import import_rig
from camera_geometry import json, transforms
from camera_geometry.camera_models import rectify_pair

from camera_geometry_ros.conversions import camera_info_msg
from camera_geometry_ros.stereo_pair import stereo_info_msg

from threading import Thread


from cv_bridge import CvBridge

from queue import Queue


def gpu_encoder():
    try:
        from nvjpeg import NvJpeg
        return NvJpeg()
    except ModuleNotFoundError:
        rospy.logwarn(f"nvjpeg not found, falling back on turbojpeg encoder")


def jpeg_encoder():
    encoder = gpu_encoder()

    if encoder is None:
        from turbojpeg import TurboJPEG
        encoder = TurboJPEG()

    return encoder


def load_config(config_file):
    if config_file is not None:
        with open(config_file) as config_file:
            return yaml.load(config_file, Loader=yaml.Loader)
    else:
        return None


class Lazy(object):
    def __init__(self, f, *args, **kwargs):
        self.f = partial(f, *args, **kwargs)
        self.result = None

    def get(self):
        self.result = self.f() if self.result is None else self.result
        return self.result


def make_crop(image, width=1200):
    image = image.get()
    h, w, *_ = image.shape
    cx, cy = (width, int((width / w) * h))

    x = (w // 2) - (cx // 2)
    y = (h // 2) - (cy // 2)
    return image[y:y + cy, x:x + cx]


def make_preview(image, width=400):
    image = image.get()
    h, w, *_ = image.shape
    height = int((width / w) * h)

    return cv2.resize(image, dsize=(width, height), interpolation=cv2.INTER_AREA)


class RawPublisher(rospy.SubscribeListener):
    def __init__(self, name, image_topic, encoding="passthrough", queue_size=4):
        super(RawPublisher, self).__init__()

        self.encoding = encoding
        self.name = name
        self.peers = {}
        self.bridge = CvBridge()

        self.publisher = rospy.Publisher(
            f"{self.name}/{image_topic}",  Image, subscriber_listener=self, queue_size=queue_size)

        self.info_publisher = rospy.Publisher(
            f"{self.name}/camera_info", CameraInfo, queue_size=queue_size)
        self.seq = 0

    def peer_subscribe(self, topic_name, topic_publish, peer_publish):
        peers = self.peers.get(topic_name, 0) + 1
        self.peers[topic_name] = peers

    def peer_unsubscribe(self, topic_name, num_peers):
        self.peers[topic_name] = num_peers

    def publish_image(self, publisher, header, lazy_image, encoding):
        if self.peers.get(publisher.name, 0) > 0:

            image_msg = self.bridge.cv2_to_imgmsg(
                lazy_image.get(), encoding=encoding)
            image_msg.header = header
            publisher.publish(image_msg)

    def publish(self, image, timestamp, cam_info=None):
        header = Header(frame_id=self.name, stamp=timestamp, seq=self.seq + 1)

        cam_info = cam_info or CameraInfo()
        cam_info.header = header

        self.info_publisher.publish(cam_info)
        self.publish_image(self.publisher, header, Lazy(
            lambda: image), encoding=self.encoding)

        self.seq += 1

    def stop(self):
        pass


class ImagePublisher(rospy.SubscribeListener):
    def __init__(self, name, raw_encoding="passthrough", queue_size=4, quality=96, preview_sizes={}):
        super(ImagePublisher, self).__init__()

        self.bridge = CvBridge()
        self.quality = quality
        self.jpeg = jpeg_encoder()

        self.raw_encoding = raw_encoding

        self.name = name
        self.peers = {}

        self.preview_sizes = preview_sizes

        self.raw_publisher = rospy.Publisher("{}/{}".format(self.name, "image_raw"),
                                             Image, subscriber_listener=self, queue_size=queue_size)

        self.color_publisher = rospy.Publisher("{}/{}".format(self.name, "image_color"),
                                               Image, subscriber_listener=self, queue_size=queue_size)

        self.compressed_publisher = rospy.Publisher("{}/{}".format(self.name, "compressed"),
                                                    CompressedImage, subscriber_listener=self, queue_size=queue_size)

        self.medium_publisher = rospy.Publisher("{}/{}".format(self.name, "medium/compressed"),
                                                CompressedImage, subscriber_listener=self, queue_size=queue_size)

        self.preview_publisher = rospy.Publisher("{}/{}".format(self.name, "preview/compressed"),
                                                 CompressedImage, subscriber_listener=self, queue_size=queue_size)

        self.centre_publisher = rospy.Publisher("{}/{}".format(self.name, "centre/compressed"),
                                                CompressedImage, subscriber_listener=self, queue_size=queue_size)

        self.info_publisher = rospy.Publisher(
            "{}/camera_info".format(self.name), CameraInfo, queue_size=queue_size)
        self.seq = 0

    def peer_subscribe(self, topic_name, topic_publish, peer_publish):
        peers = self.peers.get(topic_name, 0) + 1
        self.peers[topic_name] = peers

    def peer_unsubscribe(self, topic_name, num_peers):
        self.peers[topic_name] = num_peers

    def publish_image(self, publisher, header, lazy_image, encoding):
        if self.peers.get(publisher.name, 0) > 0:

            image_msg = self.bridge.cv2_to_imgmsg(
                lazy_image.get(), encoding=encoding)
            image_msg.header = header
            publisher.publish(image_msg)

    def publish_compressed(self, publisher, header, lazy_image):
        if self.peers.get(publisher.name, 0) > 0:
            compressed = self.jpeg.encode(lazy_image.get(), self.quality)

            image_msg = CompressedImage()
            image_msg.header = header
            image_msg.format = "jpeg"
            image_msg.data = compressed

            publisher.publish(image_msg)

    def publish(self, image, timestamp, cam_info=None):

        header = Header(frame_id=rospy.get_namespace() + self.name, stamp=timestamp, seq=self.seq + 1)

        cam_info = cam_info or CameraInfo()
        cam_info.header = header

        color_image = None
        if self.raw_encoding == "bayer_rggb8":
            color_image = Lazy(cv2.cvtColor, image, cv2.COLOR_BAYER_BG2BGR)
        elif self.raw_encoding == "bgr8":
            color_image = Lazy(lambda _: image, image)
        elif self.raw_encoding == "bayer_bggr8":
            color_image = Lazy(cv2.cvtColor, image, cv2.COLOR_BAYER_RG2BGR)
        else:
            assert False, f"TODO: implement conversion for {self.raw_encoding}"

        display_size = self.preview_sizes.get('display', 1200)

        preview_image = Lazy(make_preview, color_image, self.preview_sizes.get('preview', 400))
        medium_image = Lazy(make_preview, color_image, display_size)
        centre_image = Lazy(make_crop, color_image, display_size)

        self.info_publisher.publish(cam_info)
        self.publish_image(self.raw_publisher, header, Lazy(
            lambda: image), encoding=self.raw_encoding)
        self.publish_image(self.color_publisher, header,
                           color_image, encoding="bgr8")

        self.publish_compressed(self.compressed_publisher, header, color_image)
        self.publish_compressed(self.medium_publisher, header, medium_image)
        self.publish_compressed(self.centre_publisher, header, centre_image)
        self.publish_compressed(self.preview_publisher, header, preview_image)

        self.seq += 1

    def stop(self):
        pass


def publisher_worker(queue, publisher):
    item = queue.get()
    while item is not None:
        image, timestamp = item
        publisher.publish(image, timestamp)
        item = queue.get()


class AsyncPublisher(object):
    def __init__(self, publisher, queue_size=1):
        self.publisher = publisher

        self.queue = Queue(queue_size)
        self.thread = Thread(target=publisher_worker, args=(self.queue, self.publisher))
        self.thread.start()

    def publish(self, image, timestamp):
        self.queue.put((image, timestamp))

    def stop(self):
        self.queue.put(None)


class CalibratedPublisher(object):
    def __init__(self, name, calibration=None, **kwargs):
        self.publisher = ImagePublisher(name, **kwargs)
        self.calibration = calibration

    def publish(self, image, timestamp):
        cam_info = CameraInfo()

        if self.calibration is not None:
            # Adjust calibration to compensate for binning
            calibration = self.calibration.resize_image(
                (image.shape[1], image.shape[0]))
            cam_info = camera_info_msg(calibration)

        self.publisher.publish(image, timestamp, cam_info)

    def stop(self):
        self.publisher.stop()


def load_calibrations(namespace, calibration_file):
    rospy.loginfo(f"Loading calibrations from: {calibration_file}")

    camera_calibrations = {}
    try:
        if calibration_file is not None:
            calib = json.load_json(calibration_file)
            camera_calibrations = import_rig(calib)

            broadcaster = publish_extrinsics(
                namespace, camera_calibrations)
    except FileNotFoundError:
        rospy.logwarn(f"Calibration file not found: {calibration_file}")
    return camera_calibrations

def publish_extrinsics(namespace, cameras):

    stamp = rospy.Time.now()
    broadcaster = tf2_ros.StaticTransformBroadcaster()

    msgs = [conversions.transform_msg(camera.parent_to_camera, namespace, f"{namespace}/{child_id}", stamp)
            for child_id, camera in cameras.items()]

    broadcaster.sendTransform(msgs)

    return broadcaster
