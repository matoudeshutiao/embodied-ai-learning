#!/usr/bin/env python3

import math

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image


class SyntheticCamera(Node):
    def __init__(self):
        super().__init__("synthetic_camera")
        self.bridge = CvBridge()

        self.width = 320
        self.height = 240
        self.fx = 320.0
        self.fy = 320.0
        self.cx = self.width / 2.0
        self.cy = self.height / 2.0

        self.image_pub = self.create_publisher(
            Image, "/camera/image_raw", 10
        )
        self.depth_pub = self.create_publisher(
            Image, "/camera/depth/image_raw", 10
        )
        self.info_pub = self.create_publisher(
            CameraInfo, "/camera/camera_info", 10
        )

        self.timer = self.create_timer(0.1, self.tick)
        self.t = 0.0
        self.get_logger().info("Synthetic RGB-D camera started")

    def tick(self):
        image = np.zeros((self.height, self.width, 3), dtype=np.uint8)
        image[:] = (35, 35, 35)

        for x in range(0, self.width, 80):
            cv2.line(image, (x, 0), (x, self.height), (55, 55, 55), 1)
        for y in range(0, self.height, 80):
            cv2.line(image, (0, y), (self.width, y), (55, 55, 55), 1)

        depth = np.full(
            (self.height, self.width), 2.0, dtype=np.float32
        )

        # Red target moving in the camera optical frame.
        z = 0.35
        x = 0.15 * math.sin(self.t * 0.8)
        y = 0.04 * math.cos(self.t * 0.5)

        u = int(self.fx * x / z + self.cx)
        v = int(self.fy * y / z + self.cy)
        radius = int(self.fx * 0.05 / z)

        cv2.circle(image, (u, v), radius, (0, 0, 255), -1)
        cv2.circle(image, (u, v), radius, (0, 0, 180), 2)
        cv2.circle(depth, (u, v), radius, z, -1)

        stamp = self.get_clock().now().to_msg()

        image_msg = self.bridge.cv2_to_imgmsg(image, encoding="bgr8")
        image_msg.header.stamp = stamp
        image_msg.header.frame_id = "camera_optical_frame"

        depth_msg = self.bridge.cv2_to_imgmsg(depth, encoding="32FC1")
        depth_msg.header.stamp = stamp
        depth_msg.header.frame_id = "camera_optical_frame"

        info = CameraInfo()
        info.header.stamp = stamp
        info.header.frame_id = "camera_optical_frame"
        info.width = self.width
        info.height = self.height
        info.distortion_model = "plumb_bob"
        info.d = [0.0, 0.0, 0.0, 0.0, 0.0]
        info.k = [
            self.fx, 0.0, self.cx,
            0.0, self.fy, self.cy,
            0.0, 0.0, 1.0,
        ]
        info.r = [
            1.0, 0.0, 0.0,
            0.0, 1.0, 0.0,
            0.0, 0.0, 1.0,
        ]
        info.p = [
            self.fx, 0.0, self.cx, 0.0,
            0.0, self.fy, self.cy, 0.0,
            0.0, 0.0, 1.0, 0.0,
        ]

        self.image_pub.publish(image_msg)
        self.depth_pub.publish(depth_msg)
        self.info_pub.publish(info)

        self.t += 0.1


def main(args=None):
    rclpy.init(args=args)
    node = SyntheticCamera()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
