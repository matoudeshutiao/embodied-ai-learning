#!/usr/bin/env python3
"""Panda 桌面场景的 HSV + 深度目标检测。

输出：
  /vision/debug_image   画了检测框的 RGB 图
  /vision/target_pose   红色方块在 panda_link0 下的位姿
  /vision/target_marker RViz 中的目标球
"""

import cv2
import message_filters
import numpy as np
import rclpy
from builtin_interfaces.msg import Duration
from cv_bridge import CvBridge
from geometry_msgs.msg import PointStamped, PoseStamped
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from tf2_geometry_msgs import do_transform_point
from tf2_ros import Buffer, TransformListener
from visualization_msgs.msg import Marker


COLOR_RANGES = {
    "red": [((0, 100, 60), (10, 255, 255)),
            ((170, 100, 60), (180, 255, 255))],
    "green": [((35, 80, 50), (85, 255, 255))],
    "blue": [((100, 80, 50), (130, 255, 255))],
}
DRAW_COLORS = {
    "red": (0, 0, 255),
    "green": (0, 200, 0),
    "blue": (255, 80, 0),
}


class PandaColorDetector(Node):
    def __init__(self):
        super().__init__("panda_color_detector")
        self.bridge = CvBridge()
        self.K = None

        self.create_subscription(
            CameraInfo, "/camera/camera_info", self.on_camera_info, 10
        )
        image_sub = message_filters.Subscriber(
            self, Image, "/camera/image_raw"
        )
        depth_sub = message_filters.Subscriber(
            self, Image, "/camera/depth/image_raw"
        )
        self.sync = message_filters.ApproximateTimeSynchronizer(
            [image_sub, depth_sub], queue_size=10, slop=0.1
        )
        self.sync.registerCallback(self.on_images)

        self.debug_pub = self.create_publisher(
            Image, "/vision/debug_image", 10
        )
        self.target_pub = self.create_publisher(
            PoseStamped, "/vision/target_pose", 10
        )
        self.marker_pub = self.create_publisher(
            Marker, "/vision/target_marker", 10
        )

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.get_logger().info("Panda color detector started")

    def on_camera_info(self, msg):
        if len(msg.k) >= 9 and msg.k[0] > 0.0:
            self.K = (
                msg.k[0], msg.k[4], msg.k[2], msg.k[5]
            )

    def on_images(self, image_msg, depth_msg):
        try:
            image = self.bridge.imgmsg_to_cv2(
                image_msg, desired_encoding="bgr8"
            )
            depth = self.bridge.imgmsg_to_cv2(
                depth_msg, desired_encoding="passthrough"
            )
        except Exception as exc:
            self.get_logger().error(f"cv_bridge failed: {exc}")
            return

        if depth.dtype == np.uint16:
            depth = depth.astype(np.float32) * 0.001
        else:
            depth = depth.astype(np.float32)

        fx, fy, cx, cy = self.K or (320.0, 320.0, 160.0, 120.0)
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        debug = image.copy()
        target = None

        for color, ranges in COLOR_RANGES.items():
            mask = None
            for lower, upper in ranges:
                part = cv2.inRange(
                    hsv,
                    np.array(lower, dtype=np.uint8),
                    np.array(upper, dtype=np.uint8),
                )
                mask = part if mask is None else cv2.bitwise_or(mask, part)

            mask = cv2.morphologyEx(
                mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8)
            )
            contours, _ = cv2.findContours(
                mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
            )
            if not contours:
                continue

            contour = max(contours, key=cv2.contourArea)
            if cv2.contourArea(contour) < 50.0:
                continue

            x, y, w, h = cv2.boundingRect(contour)
            u = x + w / 2.0
            v = y + h / 2.0

            roi = np.zeros(depth.shape, dtype=np.uint8)
            cv2.drawContours(roi, [contour], -1, 255, -1)
            values = depth[roi > 0]
            values = values[np.isfinite(values) & (values > 0.0)]
            if values.size == 0:
                continue

            z = float(np.median(values))
            point_cam = PointStamped()
            point_cam.header = image_msg.header
            point_cam.point.x = (u - cx) * z / fx
            point_cam.point.y = (v - cy) * z / fy
            point_cam.point.z = z

            try:
                transform = self.tf_buffer.lookup_transform(
                    "panda_link0",
                    image_msg.header.frame_id,
                    Time(),
                )
                point_base = do_transform_point(point_cam, transform)
            except Exception as exc:
                self.get_logger().warn(f"TF failed: {exc}")
                continue

            label = f"{color} z={z:.3f}"
            cv2.rectangle(debug, (x, y), (x + w, y + h),
                          DRAW_COLORS[color], 2)
            cv2.putText(debug, label, (x, max(15, y - 5)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45,
                        DRAW_COLORS[color], 1)

            if color == "red":
                target = point_base

        if target is not None:
            pose = PoseStamped()
            pose.header.stamp = image_msg.header.stamp
            pose.header.frame_id = "panda_link0"
            pose.pose.position = target.point
            pose.pose.orientation.w = 1.0
            self.target_pub.publish(pose)
            self.publish_marker(pose)

        debug_msg = self.bridge.cv2_to_imgmsg(debug, encoding="bgr8")
        debug_msg.header = image_msg.header
        self.debug_pub.publish(debug_msg)

    def publish_marker(self, pose):
        marker = Marker()
        marker.header = pose.header
        marker.ns = "target"
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose = pose.pose
        marker.scale.x = 0.04
        marker.scale.y = 0.04
        marker.scale.z = 0.04
        marker.color.r = 1.0
        marker.color.a = 0.9
        marker.lifetime = Duration(sec=0, nanosec=200_000_000)
        self.marker_pub.publish(marker)


def main(args=None):
    rclpy.init(args=args)
    node = PandaColorDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
