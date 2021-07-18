import rospy

from queue import Queue
from threading import Thread

import numpy as np
from structs.struct import struct

from spinnaker_camera_driver_helpers.common import publish_extrinsics
from camera_geometry_ros.image_publisher import RawPublisher
from camera_geometry_ros.conversions import camera_info_msg

from sensor_msgs.msg import CameraInfo, CompressedImage, Image
from std_msgs.msg import Header

from nvjpeg_torch import Jpeg
import torch
from debayer import Debayer3x3



class CameraPublisher(rospy.SubscribeListener):
    def __init__(self, name, queue_size=4):
        super(CameraPublisher, self).__init__()

        self.name = name
        self.queue_size = queue_size
        self.peers = {}

        self.publishers = struct(
          info = self.publisher("camera_info", CameraInfo),
          raw = self.publisher("image_raw", Image),
          compressed = self.publisher("compressed", CompressedImage),
          preview = self.publisher("preview/compressed", CompressedImage)
        )

    def publisher(self, topic_name, topic_type):
      return rospy.Publisher(f"{self.name}/{topic_name}", topic_type, 
                subscriber_listener=self, queue_size=self.queue_size)


    def peer_subscribe(self, topic_name, topic_publish, peer_publish):
        peers = self.peers.get(topic_name, 0) + 1
        self.peers[topic_name] = peers

    def peer_unsubscribe(self, topic_name, num_peers):
        self.peers[topic_name] = num_peers

    @property
    def subscribed(self):
      has_subs = {k : self.peers.get(publisher.name, 0)
        for k, publisher in self.publishers.items()}
      return struct(**has_subs)

    @property 
    def has_subscribers(self, topic_name):
      assert topic_name in self.publishers
      return self.subscribed[topic_name] > 0

    def publish_image(self, topic_name, header, image, encoding):
        if self.has_subscribers(topic_name):
            image_msg = self.bridge.cv2_to_imgmsg(image, encoding=encoding)
            image_msg.header = header
            self.publishers[topic_name].publish(image_msg)

    def publish_info(self, cam_info : CameraInfo, header : Header):
      cam_info.header = header
      self.publishers.info.publish(cam_info)


    def publish_compressed(self, topic_name, data : bytes, header : Header):
      msg = CompressedImage(header = header, format = "jpeg", data = data)
      self.publishers[topic_name].publish(msg)




class SyncPublisher():
  def __init__(self, camera_names, calibrations={}, device='cuda:0', 
      publish_queue=4, preview_size=400, encoding='bayer_bggr8'):

    self.device = device
    self.publish_queue = publish_queue
    self.camera_names = camera_names

    self.preview_size = preview_size
    self.encoding = encoding

    self.publish_queue = publish_queue
    self.encoder = Jpeg()

    self.quality = 90
    self.seq = 0

    self.publishers = {k : CameraPublisher(k, queue_size=publish_queue) 
      for k in camera_names}

    self.calibrations = calibrations
    self.cam_info = {k : camera_info_msg(self.calibrations[k]) 
      if k in self.calibrations else CameraInfo()
        for k in camera_names}

    self.device = device
    self.debayer = Debayer3x3().to(dtype=torch.float16, device=self.device)

  def set_quality(self, quality):
    self.quality = quality


  def header(self, timestamp, seq):
    return Header(frame_id=self.name, stamp=timestamp, seq=seq)


  def publish(self, timestamp, images):
    with torch.no_grad():
      pinned = [ torch.from_numpy(image).pin_memory()
        for image in images.values()
      ]

      bayer = torch.stack([ image.cuda() for image in pinned ])
      rgb = self.debayer(bayer.unsqueeze(1).to(dtype=torch.float16))
      rgb = rgb.permute(0, 2, 3, 1).to(dtype=torch.uint8).contiguous()

      jpeg_data = [self.encoder.encode(image, quality=self.quality) for image in rgb]

      header = Header()
      for k, data in zip(images.keys(), jpeg_data):
        # self.publishers[k].publish_image("raw", images[k], header, encoding=self.encoding)
        self.publishers[k].publish_compressed("compressed", data.numpy().tobytes(), header)

    self.seq += 1

  def publish_extrinsics(self):
    found = {k:camera.parent_to_camera 
      for k, camera in  self.calibrations.items()}

    extrinsics = {alias:found.get(alias, np.eye(4))
      for alias in self.camera_names}

    namespace = rospy.get_namespace().strip('/')
    self.static_broadcaster = publish_extrinsics(namespace, extrinsics)    


  def stop(self):
    pass



class SyncHandler():
  def __init__(self, publisher, camera_names, sync_threshold_msec=2.0):

    self.publisher = publisher

    self.camera_names = camera_names
    self.current_frame = {}

    self.threshold_msec=sync_threshold_msec

    self.recieved_times = []
    self.queue = Queue(len(camera_names))
    self.thread = Thread(target=self.process_thread, args=())

    self.thread.start()


  def stop(self):
    rospy.loginfo("Waiting for publisher thread...")
    self.queue.put(None)
    self.thread.join()


  def set_quality(self, quality):
    self.publisher.set_quality(quality)

  def process_camera_image(self, camera_name, image, time_offset_sec):
    assert camera_name in self.camera_names
    if image.IsIncomplete():
      rospy.logwarn('Image incomplete with image status %d ...' % image.GetImageStatus())

    timestamp = image.GetTimeStamp() / 1e9 + time_offset_sec 
    frame = struct(
      timestamp = timestamp,
      frame_id = image.GetFrameID(),
      image_data = image.GetNDArray()
    )

    self.queue.put( (camera_name, frame) )
    image.Release()


  def process_thread(self):
    item = self.queue.get()
    while item is not None: 
      (camera_name, frame) = item
      self.process_frame(camera_name, frame)
      item = self.queue.get()


  def process_frame(self, camera_name, frame):
    if camera_name in self.current_frame:
        self.publish_current()

    for k, current in self.current_frame.items():
      time_diff_msec = abs(frame.timestamp - current.timestamp) * 1000.0
      print(k, time_diff_msec)
      if time_diff_msec > self.threshold_msec:
        rospy.logwarn(f"Frame time for {camera_name} differs from {k} by {time_diff_msec} ms")


    self.current_frame[camera_name] = frame
    if len(self.camera_names) == len(self.current_frame):
      self.publish_current()    

  def update_rate(self, latest):
    while len(self.recieved_times) > 0 and latest - self.recieved_times[0] > 5.0:
      self.recieved_times.pop(0)

    self.recieved_times.append(latest)

    dt = (latest - self.recieved_times[0])
    num_frames = len(self.recieved_times)

    return num_frames / dt if dt > 0 else 0



  def publish_current(self):
    assert len(self.current_frame) > 0

    frame_times = [frame.timestamp for frame in self.current_frame.values()]
    latest = min(frame_times)

    missing = set(self.camera_names) - set(self.current_frame.keys())
    if len(missing) > 0:
      rospy.logwarn(f"Incomplete trigger, missing: {missing}")

    rate = self.update_rate(latest)
    rospy.loginfo(f"{rate:.2f} Hz")

    self.publisher.publish(
      timestamp=rospy.Time.from_sec(latest), 
      images={k:frame.image_data for k, frame in self.current_frame.items()})

    self.current_frame = {}


