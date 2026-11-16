#!/usr/bin/env python3
"""收到红色目标位姿后，用 MoveIt 移动到目标上方。"""

import rclpy
from geometry_msgs.msg import Point, Pose, PoseStamped
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    BoundingVolume,
    Constraints,
    MoveItErrorCodes,
    OrientationConstraint,
    PositionConstraint,
)
from rclpy.action import ActionClient
from rclpy.node import Node
from shape_msgs.msg import SolidPrimitive


class PandaMoveToTarget(Node):
    def __init__(self):
        super().__init__("panda_move_to_target")
        self.declare_parameter("pre_grasp_offset", 0.20)
        self.offset = float(
            self.get_parameter("pre_grasp_offset").value
        )
        self.client = ActionClient(self, MoveGroup, "/move_action")
        self.create_subscription(
            PoseStamped,
            "/vision/target_pose",
            self.on_target,
            10,
        )
        self.sent = False
        self.get_logger().info(
            f"waiting for /vision/target_pose, "
            f"pre_grasp_offset={self.offset:.2f} m"
        )

    def on_target(self, msg):
        if self.sent:
            return

        if not self.client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error("/move_action action not ready")
            return

        target = PoseStamped()
        target.header = msg.header
        target.pose = msg.pose
        target.pose.position.z += self.offset

        self.get_logger().info(
            "pre-grasp target: "
            f"x={target.pose.position.x:.3f}, "
            f"y={target.pose.position.y:.3f}, "
            f"z={target.pose.position.z:.3f}"
        )

        constraint = PositionConstraint()
        constraint.header.frame_id = "panda_link0"
        constraint.link_name = "panda_link8"
        constraint.weight = 1.0

        region = BoundingVolume()
        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX
        box.dimensions = [0.01, 0.01, 0.01]
        region.primitives = [box]

        pose = Pose()
        pose.position.x = target.pose.position.x
        pose.position.y = target.pose.position.y
        pose.position.z = target.pose.position.z
        pose.orientation.w = 1.0
        region.primitive_poses = [pose]
        constraint.constraint_region = region

        orientation = OrientationConstraint()
        orientation.header.frame_id = "panda_link0"
        orientation.link_name = "panda_link8"
        orientation.orientation.x = 1.0
        orientation.orientation.w = 0.0
        orientation.absolute_x_axis_tolerance = 0.2
        orientation.absolute_y_axis_tolerance = 0.2
        orientation.absolute_z_axis_tolerance = 0.2
        orientation.weight = 1.0

        constraints = Constraints()
        constraints.position_constraints = [constraint]
        constraints.orientation_constraints = [orientation]
        constraints.name = "pre_grasp"

        goal = MoveGroup.Goal()
        goal.request.group_name = "panda_arm"
        goal.request.num_planning_attempts = 10
        goal.request.allowed_planning_time = 5.0
        goal.request.max_velocity_scaling_factor = 0.2
        goal.request.max_acceleration_scaling_factor = 0.2
        goal.request.goal_constraints = [constraints]
        goal.planning_options.plan_only = False
        goal.planning_options.replan = True
        goal.planning_options.replan_attempts = 3

        self.sent = True
        self.get_logger().info("sending MoveGroup pre-grasp request")
        future = self.client.send_goal_async(goal)
        future.add_done_callback(self.on_goal_response)

    def on_goal_response(self, future):
        handle = future.result()
        if handle is None or not handle.accepted:
            self.get_logger().error("MoveGroup goal rejected")
            return
        handle.get_result_async().add_done_callback(self.on_result)

    def on_result(self, future):
        result = future.result().result
        if result.error_code.val == MoveItErrorCodes.SUCCESS:
            self.get_logger().info("pre-grasp execution succeeded")
        else:
            self.get_logger().error(
                f"pre-grasp failed: code={result.error_code.val}"
            )


def main(args=None):
    rclpy.init(args=args)
    node = PandaMoveToTarget()
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
