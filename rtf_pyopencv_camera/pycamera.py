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

param2py = rclpy.parameter.parameter_value_to_python

#                                   1   2   4   8
ColorSpace = IntFlag("ColorSpace", "bgr rgb hsv gray")

bgr2rgb = lambda im: cv2.cvtColor(im, cv2.COLOR_BGR2RGB)
rgb2bgr = lambda im: cv2.cvtColor(im, cv2.COLOR_RGB2BGR)

hsv2bgr = lambda im: cv2.cvtColor(im, cv2.COLOR_HSV2BGR)
bgr2hsv = lambda im: cv2.cvtColor(im, cv2.COLOR_BGR2HSV)

bgr2gray = lambda im: cv2.cvtColor(im, cv2.COLOR_BGR2GRAY)
gray2bgr = lambda im: cv2.cvtColor(im, cv2.COLOR_GRAY2BGR)

rgb2gray = lambda im: cv2.cvtColor(im, cv2.COLOR_RGB2GRAY)
gray2rgb = lambda im: cv2.cvtColor(im, cv2.COLOR_GRAY2RGB)

camera240 = [240,320]
camera480 = [480,640]
camera720 = [720,1280]
camera1080 = [1080,1920]

# class CameraSize(IntFlag):
#     320  = 0
#     480  = 1
#     720  = 2
#     1080 = 4

class rtf_camera(Node):
    def __init__(self, camera_num=0, frame_id="camera", i2c=None):
        super().__init__('rtf_camera')

        self.frame_id = frame_id # self.declare_parameter('frame_id', "camera").value
        self.camera_num = camera_num # self.declare_parameter("camera_num", camera_num).value

        self.encoding = self.declare_parameter("encoding", "mono8").value
        self.rectify = self.declare_parameter("rectify", False).value
        self.camera_size = self.declare_parameter("camera_size", 240).value

        self.handler = ParameterEventHandler(self)

        self.callback_handle_size = self.handler.add_parameter_callback(
            parameter_name="camera_size",
            node_name=self.get_name(),
            callback=self.set_size,
        )

        self.callback_handle_rect = self.handler.add_parameter_callback(
            parameter_name="rectify",
            node_name=self.get_name(),
            callback=self.set_rectify,
        )

        self.callback_handle_enc = self.handler.add_parameter_callback(
            parameter_name="encoding",
            node_name=self.get_name(),
            callback=self.set_encoding,
        )

        self.camera = cv2.VideoCapture(camera_num)

        self.timer = self.create_timer(1/30, self.callback)
        self.pub_im = self.create_publisher(Image, 'image', 10)
        self.pub_cim = self.create_publisher(CompressedImage, 'image_compressed', 10)
        self.pub_ci = self.create_publisher(CameraInfo, 'camera_info', 10)

        self.cameraInfo = self.yaml_to_CameraInfo()
        # self.cameraInfo.header.frame_id = self.frame_id

        self.logger = self.get_logger()
        self.print_info()

    def print_info(self):
        width = int(self.camera.get(cv2.CAP_PROP_FRAME_WIDTH))
        height = int(self.camera.get(cv2.CAP_PROP_FRAME_HEIGHT))
        fps = self.camera.get(cv2.CAP_PROP_FPS)

        self.logger.info("==================")
        self.logger.info(f"Camera: {self.camera_num}")
        self.logger.info(f"Image size:: {self.camera_size} [{height}x{width}]")
        self.logger.info(f"Colorspace: {self.encoding}")
        # self.logger.info(f"FPS: {fps}")
        self.logger.info(f"Recified: {self.rectify}")
        self.logger.info(f"Camera Info: {self.cameraInfo}")

    def yaml_to_CameraInfo(fname=None):
        try:
            with open(fname, "r") as fd:
                # Load data from yaml file
                calib_data = yaml.load(fd)

                # Parse
                cameraInfo_msg = CameraInfo()
                cameraInfo.header.frame_id = self.frame_id
                cameraInfo_msg.width = calib_data["image_width"]
                cameraInfo_msg.height = calib_data["image_height"]
                cameraInfo_msg.distortion_model = calib_data["distortion_model"]
                cameraInfo_msg.K = calib_data["camera_matrix"]["data"]
                cameraInfo_msg.D = calib_data["distortion_coefficients"]["data"]
                cameraInfo_msg.R = calib_data["rectification_matrix"]["data"]
                cameraInfo_msg.P = calib_data["projection_matrix"]["data"]
                return cameraInfo_msg
        except:
            print(f"{Fore.RED}*** Invalid YAML file: {fname} ***{Fore.RESET}")
            return None

    def set_encoding(self, p):
        self.encoding = param2py(p.value)
        self.get_logger().info(f">> param {p.name}: {self.encoding}")
        self.print_info()

    def set_rectify(self, p):
        val = param2py(p.value)
        self.rectify = val
        self.get_logger().info(f">> param {p.name}: {self.encoding}")
        self.print_info()

    def set_size(self, p):
        camera_size = param2py(p.value)
        self.get_logger().info(f">> param: {p.name}: {camera_size}")
        self.camera_size = camera_size

        height, width = camera480
        if camera_size == 240: height, width = camera320
        elif camera_size == 480: height, width = camera480
        elif camera_size == 720: height, width = camera720
        elif camera_size == 1080: height, width = camera1080

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

        s = frame.shape

        # try:
        stamp = self.get_clock().now().to_msg()
        msg = Image()
        msg.header.stamp = stamp

        if self.encoding == "mono8":
            frame = bgr2gray(frame)
            msg.encoding = "mono8"
            msg.step = s[1]
        elif self.encoding == "bgr8":
            # frame = bgr2gray(frame)
            msg.encoding = "bgr8"
            msg.step = s[1]*3
        elif self.encoding == "rgb8":
            frame = bgr2rgb(frame)
            msg.encoding = "rgb8"
            msg.step = s[1]*3
        else:
            frame = bgr2gray(frame)
            msg.encoding = "mono8"
            msg.step = s[1]


        if self.cameraInfo is not None:
            # publish rectified image
            # FIXME: this is rect yet!!
            # self.pub_imr.publish(self.msg)
            if self.rectify is True:
                pass

            self.cameraInfo.header.stamp = stamp
            self.pub_ci.publish(self.cameraInfo)

        data = frame.ravel().tolist()
        # data = np.array(frame).tobytes()
        # encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 90]
        # result, encimg = cv2.imencode('.jpg', img, encode_param)
        # ok, data = cv2.imencode('.jpg', frame)

        msg.header.frame_id = self.frame_id
        msg.width = s[1]
        msg.height = s[0]
        msg.data = data
        self.pub_im.publish(msg)

        cim = CompressedImage()
        cim.header.stamp = stamp
        cim.header.frame_id = self.frame_id
        cim.format = "jpeg"
        encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 90]
        _, cdata = cv2.imencode('.jpg', frame, encode_param)
        cim.data = np.array(cdata).tostring()
        self.pub_cim.publish(cim)

        # if self.cameraInfo is None:
        #     return

        # # print(self.cameraInfo)

        # # publish rectified image
        # # FIXME: this is rect yet!!
        # self.pub_imr.publish(self.msg)

        # self.cameraInfo.header.stamp = stamp
        # # if self.cameraInfo.width == 0: self.cameraInfo.width = s[1]
        # # if self.cameraInfo.height == 0: self.cameraInfo.height = s[0]
        # self.pub_ci.publish(self.cameraInfo)

        # # except Exception as e:
        # #     print(f"{Fore.RED}*** {e} ***{Fore.RESET}")

def main(args=None):
    rclpy.init(args=args)

    camera = rtf_camera()

    try:
        rclpy.spin(camera)
    except KeyboardInterrupt:
        print("bye ...")
    finally:
        camera.destroy_node()
        # rclpy.shutdown()



if __name__ == '__main__':
    main()
