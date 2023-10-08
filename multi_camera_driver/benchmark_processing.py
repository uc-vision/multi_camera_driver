#!/usr/bin/env python3

import argparse
import threading
from typing import List
import rospy2 as rospy
from multi_camera_driver.helpers.work_queue import WorkQueue
import torch

from tqdm import tqdm
import numpy as np
from multi_camera_driver.helpers.common import CameraImage, CameraSettings, ImageEncoding
from multi_camera_driver.helpers.image_processor.outputs import ImageOutputs
from multi_camera_driver.helpers.image_settings import ImageSettings

from multi_camera_driver.helpers.image_processor import FrameProcessor, TiQueue

from taichi_image import bayer
from taichi_image.test.camera_isp import load_test_image

import taichi as ti
import cv2


def main():
  parser = argparse.ArgumentParser()
  parser.add_argument("filename", help="Path to image file")
  parser.add_argument("--device", default="cuda", help="Device to use for processing")
  parser.add_argument("--resize_width", type=int, default=0, help="Resize width")
  parser.add_argument("--transform", type=str, default='none', help="Transformation to apply")
  parser.add_argument("--preload", action="store_true", help="Preload imges to cuda")
  parser.add_argument("--n", type=int, default=6, help="Number of cameras to test")
  parser.add_argument("--frames", type=int, default=300, help="Number of cameras to test")
  parser.add_argument("--no_compress", action="store_true", help="Disable compression")

  args = parser.parse_args()


  print(args)
  image_settings= ImageSettings(
      device=args.device,
      jpeg_quality=96,
      preview_size=200,
      resize_width=args.resize_width,
      tone_mapping="reinhard",
      tone_gamma= 1.0,
      tone_intensity= 1.0,
      color_adapt=0.0,
      light_adapt=0.5,
      transform=args.transform
  )

  test_packed, test_image  = TiQueue.run_sync(load_test_image, args.filename, bayer.BayerPattern.RGGB)
  test_packed = torch.from_numpy(test_packed)

  if args.preload:
      test_packed = test_packed.cuda()

  h, w, _ = test_image.shape


  encoding = ImageEncoding.Bayer_BGGR12
  camera_settings = {f"{n}":CameraSettings(
      name="test{n}",
      serial="{n}",
      connection_speed="SuperSpeed",
      encoding = encoding,
      image_size=(w, h),
      framerate=30.0,
      master_id="test0",
      time_offset_sec=rospy.Duration(0))

  for n in range(args.n)}

  frame_processor = FrameProcessor(camera_settings, image_settings)


  pbar = tqdm(total=int(args.frames))

  def on_frame(outputs):
    if not args.no_compress:

      for output in outputs:
        compressed = output.compressed
        preview = output.compressed_preview

    pbar.update(1)

  processor = WorkQueue("publisher", run=on_frame, num_workers=4, max_size=4)
  processor.start()


  images = {f"{n}":CameraImage(
    camera_name=f"test{n}",
    image_data=test_packed.clone(),
    seq=0,
    image_size=(w, h),
    encoding=encoding,
    timestamp=rospy.Time(),
    clock_time=rospy.Time())

    for n in range(6) }

  frame_processor.bind(on_frame=on_frame)
      
  for _ in range(int(args.frames)):

    # frame_processor.settings.tone_intensity += 0.01
    frame_processor.process(images)

  frame_processor.stop()
  processor.stop()

  TiQueue.stop()
  print("Finished")

if __name__ == "__main__":
  with torch.inference_mode():
    main()
