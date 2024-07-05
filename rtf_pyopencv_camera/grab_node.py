# %YAML:1.0
# ---
# image_width: 640
# image_height: 360
# K: !!opencv-matrix
#    rows: 3
#    cols: 3
#    dt: d
#    data: [259.24853515625, 0., 327.3691101074219, 0.,
#           259.24853515625, 178.34190368652344, 0., 0., 1.]
# D: !!opencv-matrix
#    rows: 1
#    cols: 8
#    dt: d
#    data: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
# R: !!opencv-matrix
#    rows: 3
#    cols: 3
#    dt: d
#    data: [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
# P: !!opencv-matrix
#    rows: 3
#    cols: 4
#    dt: d
#    data: [259.24853515625, 0., 327.3691101074219, 0.,
#           0., 259.24853515625, 178.34190368652344, 0.,
#           0., 0., 1.0, 0.]

# -*- coding: utf-8 -*-
##############################################
# The MIT License (MIT)
# Copyright (c) 2020 Kevin Walchko
# see LICENSE for full details
##############################################

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage
from sensor_msgs.msg import CameraInfo
import rclpy.parameter
from rclpy.parameter_event_handler import ParameterEventHandler

import yaml
from colorama import Fore
import cv2
from enum import IntFlag
import numpy as np
from pathlib import Path
import time

from .common import *
from rtf_interfaces.srv import CaptureImage
from std_srvs.srv import Empty


# import sys, select, os
# # if os.name == 'nt':
# #   import msvcrt, time
# # else:
# import tty, termios

# settings = termios.tcgetattr(sys.stdin)

# def getKey():
#     tty.setraw(sys.stdin.fileno())
#     rlist, _, _ = select.select([sys.stdin], [], [], 0.01)
#     if rlist:
#         print(f"rlist: {rlist}")
#         key = sys.stdin.read(1)
#     else:
#         key = ''

#     if key == '\x03':
#         print("ctrl+C ... bye")
#         sys.exit(0)
#     else:
#         termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

#     return key

# shorten this call
param2py = rclpy.parameter.parameter_value_to_python

# #                                   1   2   4   8
# ColorSpace = IntFlag("ColorSpace", "bgr rgb hsv gray")

# bgr2rgb = lambda im: cv2.cvtColor(im, cv2.COLOR_BGR2RGB)
# rgb2bgr = lambda im: cv2.cvtColor(im, cv2.COLOR_RGB2BGR)

# hsv2bgr = lambda im: cv2.cvtColor(im, cv2.COLOR_HSV2BGR)
# bgr2hsv = lambda im: cv2.cvtColor(im, cv2.COLOR_BGR2HSV)

# bgr2gray = lambda im: cv2.cvtColor(im, cv2.COLOR_BGR2GRAY)
# gray2bgr = lambda im: cv2.cvtColor(im, cv2.COLOR_GRAY2BGR)

# rgb2gray = lambda im: cv2.cvtColor(im, cv2.COLOR_RGB2GRAY)
# gray2rgb = lambda im: cv2.cvtColor(im, cv2.COLOR_GRAY2RGB)

# camera240 = [240,320]
# camera480 = [480,640]
# camera720 = [720,1280]
# camera1080 = [1080,1920]

# class CameraSize(IntFlag):
#     320  = 0
#     480  = 1
#     720  = 2
#     1080 = 4

class rtf_camera(Node):
    def __init__(self, camera_num=0, frame_id="camera", i2c=None):
        super().__init__('grab_node')

        self.frame_id = frame_id # self.declare_parameter('frame_id', "camera").value
        self.camera_num = camera_num # self.declare_parameter("camera_num", camera_num).value
        self.camera_size = self.declare_parameter("camera_size", 720).value

        self.handler = ParameterEventHandler(self)

        self.callback_handle_size = self.handler.add_parameter_callback(
            parameter_name="camera_size",
            node_name=self.get_name(),
            callback=self.set_size_cb,
        )
        # self.srv = self.create_service(CaptureImage, 'capture', self.capture_cb)
        self.srv = self.create_service(Empty, 'capture', self.capture_cb)

        # cv2.CAP_MODE_BGR - BGR24 (default)
        # cv2.CAP_MODE_RGB - RGB24
        # cv2.CAP_MODE_GRAY - Y8
        # cv2.CAP_MODE_YUYV - YUYV
        self.camera = cv2.VideoCapture(camera_num)
        # self.possible = self.camera.set(cv2.CAP_PROP_MODE, cv2.CAP_MODE_GRAY)
        # print(self.possible)
        # self.possible = False

        self.timer = self.create_timer(1/30, self.callback)
        self.pub_im = self.create_publisher(Image, 'image', 10)

        self.logger = self.get_logger()
        self.set_size(self.camera_size)
        self.print_info()

        self.imgs = [] # store captured images
        self.capture = False

    def capture_cb(self, req, resp):
        self.capture = True
        # resp.status = "good"
        return resp

    def print_info(self):
        width = int(self.camera.get(cv2.CAP_PROP_FRAME_WIDTH))
        height = int(self.camera.get(cv2.CAP_PROP_FRAME_HEIGHT))
        # fps = self.camera.get(cv2.CAP_PROP_FPS)

        self.logger.info("==================")
        self.logger.info(f"Camera: {self.camera_num}")
        self.logger.info(f"Image size:: {self.camera_size} [{height}x{width}]")
        # self.logger.info(f"Colorspace: {self.encoding}")
        # self.logger.info(f"FPS: {fps}")
        # self.logger.info(f"Recified: {self.rectify}")
        # self.logger.info(f"Camera Info: {self.cameraInfo}")

    def set_size_cb(self, p):
        camera_size = param2py(p.value)
        self.get_logger().info(f">> param: {p.name}: {camera_size}")
        self.set_size(camera_size)

    def set_size(self, camera_size):
        self.camera_size = camera_size

        height, width = camera480
        if camera_size == 240: height, width = camera320
        elif camera_size == 480: height, width = camera480
        elif camera_size == 720: height, width = camera720
        elif camera_size == 1080: height, width = camera1080
        else:
            self.logger.error(f"Invalid size: {camera_size}")
            return

        self.camera.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        self.camera.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        self.print_info()

    def callback(self):
        try:
            ok, frame = self.camera.read()
            if not ok:
                raise Exception("Couldn't read from camera")

        except Exception as e:
            print(f"{Fore.RED}*** {e} ***{Fore.RESET}")
            return

        # if not self.possible:
        frame = bgr2gray(frame)

        h,w = frame.shape

        data = frame.ravel().tolist()
        # data = np.array(frame).tobytes()

        stamp = self.get_clock().now().to_msg()
        msg = Image()
        msg.header.stamp = stamp
        msg.header.frame_id = self.frame_id
        msg.width = w
        msg.height = h
        msg.step = w
        msg.data = data
        self.pub_im.publish(msg)

        # ch = cv2.waitKey(2)
        # ch = getKey()

        # Quit program using ESC or q
        # if ch in [27, ord('q')]:
        #     # if video:
        #     #     save.release()
        #     break

        # Capture a single frame
        # if ch == 's':
        if self.capture:
            self.imgs.append(frame)
            print(f">> captured image {len(self.imgs)}")
            self.capture = False


def main(args=None):
    rclpy.init(args=args)

    camera = rtf_camera()

    try:
        rclpy.spin(camera)
    except KeyboardInterrupt:
        print("KeyboardInterrupt ...")
    except SystemExit:
        print("SystemExit ...")
    finally:
        p = Path(".")
        path = p.expanduser().absolute()

        if len(camera.imgs) > 0:
            shot_idx = 0
            # printInfo(f"Saving images to {str(path)}")
            for img in camera.imgs:
                fn = '{0:03d}.png'.format(shot_idx)
                p = str(path.joinpath(fn))
                cv2.imwrite(p, img)
                shot_idx += 1
            print(f"Wrote {len(camera.imgs)} images to {str(path)}")

        camera.destroy_node()
        # rclpy.shutdown()
        # termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)



if __name__ == '__main__':
    main()
