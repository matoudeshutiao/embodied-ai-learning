#!/usr/bin/env python3
"""把 Gazebo 桌面场景同步到 MoveIt planning scene。"""

import rclpy
from geometry_msgs.msg import Pose
from moveit_msgs.msg import CollisionObject, PlanningScene
from rclpy.node import Node
from shape_msgs.msg import SolidPrimitive


class PandaScene(Node):
    def __init__(self):
        super().__init__("panda_scene")
        self.pub = self.create_publisher(
            PlanningScene, "/planning_scene", 10
        )
        self.sent = False
        self.publish_count = 0
        self.timer = self.create_timer(1.0, self.publish_scene)

    @staticmethod
    def box(object_id, position, size):
        obj = CollisionObject()
        obj.header.frame_id = "panda_link0"
        obj.id = object_id
        obj.operation = CollisionObject.ADD

        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.BOX
        primitive.dimensions = list(size)

        pose = Pose()
        pose.position.x, pose.position.y, pose.position.z = position
        pose.orientation.w = 1.0

        obj.primitives = [primitive]
        obj.primitive_poses = [pose]
        return obj

    def publish_scene(self):
        scene = PlanningScene()
        scene.is_diff = True
        scene.robot_state.is_diff = True
        scene.world.collision_objects = [
            self.box("table", (0.55, 0.0, 0.175), (0.80, 0.80, 0.35)),
            self.box("red_cube", (0.45, -0.15, 0.375), (0.05, 0.05, 0.05)),
            self.box("green_cube", (0.55, 0.0, 0.375), (0.05, 0.05, 0.05)),
            self.box("blue_cube", (0.65, 0.15, 0.375), (0.05, 0.05, 0.05)),
            self.box("blue_box", (0.55, 0.24, 0.38), (0.16, 0.16, 0.10)),
        ]
        self.pub.publish(scene)
        self.publish_count += 1
        if not self.sent:
            self.get_logger().info("published table scene to MoveIt")
            self.sent = True
        if self.publish_count >= 3:
            self.timer.cancel()


def main(args=None):
    rclpy.init(args=args)
    node = PandaScene()
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
