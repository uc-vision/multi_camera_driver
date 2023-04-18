#!/usr/bin/env python3

import argparse
from typing import List
import rospy
from spinnaker_camera_driver_helpers.work_queue import WorkQueue

from tqdm import tqdm
import numpy as np
from spinnaker_camera_driver_helpers.common import CameraImage, CameraSettings, ImageEncoding
from spinnaker_camera_driver_helpers.image_processor.outputs import ImageOutputs
from spinnaker_camera_driver_helpers.image_settings import ImageSettings

from spinnaker_camera_driver_helpers.image_processor import FrameProcessor, TiQueue

from taichi_image import bayer, packed

import cv2


def main():
  parser = argparse.ArgumentParser()
  parser.add_argument("filename", help="Path to image file")
  parser.add_argument("--device", default="cuda", help="Device to use for processing")
  parser.add_argument("--resize_width", type=int, default=0, help="Resize width")
  parser.add_argument("--n", type=int, default=6, help="Number of cameras to test")
  parser.add_argument("--frames", type=int, default=300, help="Number of cameras to test")
  
  

  args = parser.parse_args()
  print(args)

  image_settings= ImageSettings(
      device=args.device,
      jpeg_quality=95,
      preview_size=200,
      resize_width=args.resize_width,
      tone_mapping="reinhard"
  )

  image = cv2.imread(args.filename)
  bayer_image = TiQueue.run_sync(bayer.rgb_to_bayer, cv2.cvtColor(image, cv2.COLOR_BGR2RGB), 
                                   bayer.BayerPattern.BGGR)
  
  # bayer_image16 = bayer_image.astype(np.uint16) * 256
  # bayer_image16 = bayer_image.astype(np.float16) / 256
  packed12_image = TiQueue.run_sync(packed.encode12, bayer_image.reshape(-1)) 

  h, w = image.shape[:2]
  encoding = ImageEncoding.Bayer_BGGR12
  camera_settings = {f"{n}":CameraSettings(
      name="test{n}",
      serial="{n}",
      connection_speed="SuperSpeed",
      encoding = encoding,
      image_size=(w, h),
      is_master=False,
      is_free_running=True,
      max_framerate=30.0,
      time_offset_sec=rospy.Duration(0))

  for n in range(args.n)}

  frame_processor = FrameProcessor(camera_settings, image_settings)
  images = {f"{n}":CameraImage(
    camera_name=f"test{n}",
    image_data=packed12_image.copy(),
    seq=0,
    image_size=(w, h),
    encoding=encoding,
    timestamp=rospy.Time())

    for n in range(int(args.n)) }


  pbar = tqdm(total=int(args.frames))
  n_bytes = 0 

  def on_frame(outputs:List[ImageOutputs]):
    nonlocal n_bytes

    for output in outputs:
      compressed = output.compressed
      preview = output.compressed_preview
      n_bytes += len(compressed) + len(preview)

    pbar.update(1)

  processor = WorkQueue("publisher", run=on_frame)
  processor.start()
  frame_processor.bind(on_frame=processor.enqueue)
      
  for _ in range(int(args.frames)):
    frame_processor.process(images)

  frame_processor.stop()
  processor.stop()

  print("Finished")

if __name__ == "__main__":
  main()
