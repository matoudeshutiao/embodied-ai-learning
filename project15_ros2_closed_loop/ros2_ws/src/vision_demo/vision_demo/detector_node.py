#!/usr/bin/env python3

import cv2
import message_filters
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import PointStamped, PoseStamped
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import Image
from tf2_geometry_msgs import do_transform_point
from tf2_ros import Buffer, TransformListener
from vision_msgs.msg import (
    Detection3D,
    Detection3DArray,
    ObjectHypothesisWithPose,
)
from visualization_msgs.msg import Marker


class ColorDetector(Node):
    def __init__(self):
        super().__init__("color_detector")
        self.bridge = CvBridge()

        self.image_sub = message_filters.Subscriber(
            self, Image, "/camera/image_raw"
        )
        self.depth_sub = message_filters.Subscriber(
            self, Image, "/camera/depth/image_raw"
        )

        self.sync = message_filters.ApproximateTimeSynchronizer(
            [self.image_sub, self.depth_sub],
            queue_size=10,
            slop=0.05,
        )
        self.sync.registerCallback(self.callback)

        self.det_pub = self.create_publisher(
            Detection3DArray, "/vision/detections", 10
        )
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

        # Match the 320x240 synthetic camera intrinsics.
        self.fx = 320.0
        self.fy = 320.0
        self.cx = 160.0
        self.cy = 120.0

        self.get_logger().info("Color detector started")

    def callback(self, image_msg, depth_msg):
        try:
            image = self.bridge.imgmsg_to_cv2(
                image_msg, desired_encoding="bgr8"
            )
            depth = self.bridge.imgmsg_to_cv2(
                depth_msg, desired_encoding="32FC1"
            )
        except Exception as exc:
            self.get_logger().error(f"cv_bridge failed: {exc}")
            return

        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        mask1 = cv2.inRange(hsv, (0, 100, 80), (10, 255, 255))
        mask2 = cv2.inRange(hsv, (170, 100, 80), (180, 255, 255))
        mask = cv2.bitwise_or(mask1, mask2)
        mask = cv2.morphologyEx(
            mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8)
        )

        contours, _ = cv2.findContours(
            mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )

        detections = Detection3DArray()
        detections.header = image_msg.header

        debug = image.copy()

        for contour in contours:
            area = cv2.contourArea(contour)
            if area < 50.0:
                continue

            x, y, w, h = cv2.boundingRect(contour)
            u = x + w / 2.0
            v = y + h / 2.0

            mask_roi = np.zeros(depth.shape, dtype=np.uint8)
            cv2.drawContours(mask_roi, [contour], -1, 255, -1)
            depth_values = depth[mask_roi > 0]
            depth_values = depth_values[
                np.isfinite(depth_values) & (depth_values > 0.0)
            ]
            if depth_values.size == 0:
                continue

            z = float(np.median(depth_values))
            X = (u - self.cx) * z / self.fx
            Y = (v - self.cy) * z / self.fy

            det = Detection3D()
            det.header = image_msg.header
            det.bbox.center.position.x = X
            det.bbox.center.position.y = Y
            det.bbox.center.position.z = z
            det.bbox.center.orientation.w = 1.0
            det.bbox.size.x = float(w) * z / self.fx
            det.bbox.size.y = float(h) * z / self.fy
            det.bbox.size.z = 0.05

            hypothesis = ObjectHypothesisWithPose()
            hypothesis.hypothesis.class_id = "red_object"
            hypothesis.hypothesis.score = 1.0
            hypothesis.pose.pose.position.x = X
            hypothesis.pose.pose.position.y = Y
            hypothesis.pose.pose.position.z = z
            hypothesis.pose.pose.orientation.w = 1.0

            det.results.append(hypothesis)
            detections.detections.append(det)

            point_cam = PointStamped()
            point_cam.header = image_msg.header
            point_cam.point.x = X
            point_cam.point.y = Y
            point_cam.point.z = z

            try:
                transform = self.tf_buffer.lookup_transform(
                    "base_link",
                    image_msg.header.frame_id,
                    Time(),
                )
                point_base = do_transform_point(point_cam, transform)
            except Exception as exc:
                self.get_logger().warn(f"TF transform failed: {exc}")
                continue

            target = PoseStamped()
            target.header.stamp = image_msg.header.stamp
            target.header.frame_id = "base_link"
            target.pose.position.x = point_base.point.x
            target.pose.position.y = point_base.point.y
            target.pose.position.z = point_base.point.z
            target.pose.orientation.w = 1.0
            self.target_pub.publish(target)

            marker = Marker()
            marker.header.stamp = image_msg.header.stamp
            marker.header.frame_id = "base_link"
            marker.ns = "vision"
            marker.id = 0
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position.x = point_base.point.x
            marker.pose.position.y = point_base.point.y
            marker.pose.position.z = point_base.point.z
            marker.pose.orientation.w = 1.0
            marker.scale.x = 0.05
            marker.scale.y = 0.05
            marker.scale.z = 0.05
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            marker.color.a = 1.0
            marker.lifetime = Duration(seconds=0.5).to_msg()
            self.marker_pub.publish(marker)

            cv2.rectangle(
                debug, (x, y), (x + w, y + h), (0, 255, 0), 2
            )
            cv2.circle(debug, (int(u), int(v)), 4, (255, 0, 0), -1)
            cv2.putText(
                debug,
                f"red z={z:.2f}m",
                (x, max(15, y - 8)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (0, 255, 0),
                1,
            )

        self.det_pub.publish(detections)

        debug_msg = self.bridge.cv2_to_imgmsg(debug, encoding="bgr8")
        debug_msg.header = image_msg.header
        self.debug_pub.publish(debug_msg)


def main(args=None):
    rclpy.init(args=args)
    node = ColorDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
