# -*- coding: utf-8 -*-
##############################################
# The MIT License (MIT)
# Copyright (c) 2020 Kevin Walchko
# see LICENSE for full details
##############################################

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from colorama import Fore
import cv2

class rtf_camera(Node):
    def __init__(self, i2c=None):
        super().__init__('rtf_camera')

        self.camera = cv2.VideoCapture(0)

        self.timer = self.create_timer(1/30, self.callback)

        self.pub = self.create_publisher(Image, 'image', 10)

        frame_id = self.declare_parameter('frame_id', "base_imu_link").value

        self.msg = Image()
        self.msg.header.frame_id = frame_id
        self.msg.encoding = "mono8"

    def callback(self):
        try:
            ok, frame = self.camera.read()
            if not ok:
                raise Exception("Couldn't read from camera")

        except Exception as e:
            print(f"{Fore.RED}*** {e} ***{Fore.RESET}")
            return

        try:
            stamp = self.get_clock().now().to_msg()
            self.msg.header.stamp = stamp
            frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            s = frame.shape
            self.msg.width = s[1]
            self.msg.step = s[1]
            self.msg.height = s[0]
            self.msg.data = frame.ravel().tolist()
            self.pub.publish(self.msg)

        except Exception as e:
            print(f"{Fore.RED}*** {e} ***{Fore.RESET}")

def main(args=None):
    rclpy.init(args=args)

    camera = rtf_camera()

    try:
        rclpy.spin(camera)
    except KeyboardInterrupt:
        print("bye ...")
    finally:
        camera.destroy_node()
        rclpy.shutdown()



if __name__ == '__main__':
    main()
