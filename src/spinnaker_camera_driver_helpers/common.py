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
from maara_msgs.msg import StereoCameraInfo

from camera_geometry.camera import Camera, Transform

import camera_geometry_ros.conversions as conversions
from camera_geometry.import_calibration import import_cameras
from camera_geometry import image_utils, json, util, stereo_pair

from camera_geometry_ros.conversions import camera_info_msg
from camera_geometry_ros.stereo_pair import stereo_info_msg, stereo_pair_from_msg

from threading import Thread
from turbojpeg import TurboJPEG
from cv_bridge import CvBridge

from queue import Queue


def load_config(config_file):
    if config_file is not None:
        with open(config_file) as config_file:
            return yaml.load(config_file, Loader=yaml.Loader)    
    else:
        return None


def load_calibration(calibration_file):
    if calibration_file is not None:
        calib_data = json.load_json(calibration_file)
        cameras, extrinsics = import_cameras(calib_data)

        return cameras, extrinsics, calib_data['stereo_pairs']
    else:
        return {}, {}, {}


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
            queue_size=queue_size) for camera in ["left", "right"]]

        image_subscribers = []

        image_subscribers = [message_filters.Subscriber(topic, Image, queue_size=queue_size, buff_size=2**24) for topic in image_topics]        
        info_subscribers = [message_filters.Subscriber(topic, CameraInfo, queue_size=queue_size) for topic in info_topics]   
               
        sync = message_filters.TimeSynchronizer(info_subscribers + image_subscribers, queue_size=1)
        sync.registerCallback(self.frame_callback)

        self.broadcaster = tf2_ros.StaticTransformBroadcaster()



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

            timestamp = left_info.header.stamp

            if self.pair is None or (not equal_pair(self.pair, left, right)):
                self.pair = stereo_pair.rectify_pair(left, right)
    
                # Broadcast the rectification induced rotation as a static transform
                transform = Transform(self.frames[0], rotation=self.pair.left.rotation)
                msg = conversions.transform_msg(transform, self.name + "/left", timestamp)
                self.broadcaster.sendTransform(msg)

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


class Lazy(object):
    def __init__(self, f, *args, **kwargs):
        self.f = partial(f, *args, **kwargs)
        self.result = None

    def get(self):
        self.result = self.f() if self.result is None else self.result
        return self.result

def make_crop(image, image_scale=0.1):
    image = image.get()
    h, w, *_ = image.shape
    cx, cy = int(w * image_scale), int(h * image_scale)
    x = (w // 2) - (cx // 2)
    y = (h // 2) - (cy // 2)    
    return image[y:y + cy, x:x + cx]

def make_preview(image, image_scale=0.1):
    image = image.get()
    h, w, *_ = image.shape
    preview_size = (int(w * image_scale), int(h * image_scale))
    return cv2.resize(image, dsize=preview_size, interpolation=cv2.INTER_LINEAR)


class ImagePublisher(rospy.SubscribeListener):
    def __init__(self, name, raw_encoding="passthrough", queue_size=4, quality=96):
        super(ImagePublisher, self).__init__()

        self.bridge = CvBridge()
        self.jpeg = TurboJPEG()
        self.quality = quality

        self.raw_encoding = raw_encoding

        self.name = name
        self.peers = {}

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

        self.info_publisher = rospy.Publisher("{}/camera_info".format(self.name), CameraInfo, queue_size=queue_size)
        self.seq = 0

    def peer_subscribe(self, topic_name, topic_publish, peer_publish):
        peers = self.peers.get(topic_name, 0) + 1
        self.peers[topic_name] = peers 

    def peer_unsubscribe(self, topic_name, num_peers):
        self.peers[topic_name] = num_peers

    def publish_image(self, publisher, header, lazy_image, encoding):
        if self.peers.get(publisher.name, 0) > 0:

            image_msg = self.bridge.cv2_to_imgmsg(lazy_image.get(), encoding=encoding)
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
        header = make_header(self.name, timestamp, self.seq + 1)

        cam_info = cam_info or CameraInfo()        
        cam_info.header = header
        
        color_image = None
        if self.raw_encoding == "bayer_rggb8":
          color_image = Lazy(cv2.cvtColor, image, cv2.COLOR_BAYER_BG2BGR)
        elif self.raw_encoding == "bgr8":
          color_image = Lazy(lambda _: image, image)
        else:
          assert False, f"TODO: implement conversion for {self.raw_encoding}"
        
        preview_image = Lazy(make_preview, color_image, 0.1)
        medium_image = Lazy(make_preview, color_image, 1/3.0)
        centre_image = Lazy(make_crop, color_image, 1/3.0)


        self.info_publisher.publish(cam_info)
        self.publish_image(self.raw_publisher, header, Lazy(lambda: image), encoding=self.raw_encoding)     
        self.publish_image(self.color_publisher, header, color_image, encoding="bgr8")     

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
            calibration = self.calibration.resize_image((image.shape[1], image.shape[0]))
            cam_info = camera_info_msg(calibration)

        self.publisher.publish(image, timestamp, cam_info)

    def stop(self):
        self.publisher.stop()

def publish_extrinsics(extrinsics):

    stamp = rospy.Time.now()
    broadcaster = tf2_ros.StaticTransformBroadcaster()

    for child_id, transform in extrinsics.items():      
        msg = conversions.transform_msg(transform, child_id, stamp)
        broadcaster.sendTransform(msg)
    return broadcaster