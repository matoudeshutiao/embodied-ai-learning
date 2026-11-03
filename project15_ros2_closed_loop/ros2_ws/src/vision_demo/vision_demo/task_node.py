#!/usr/bin/env python3

import rclpy
from geometry_msgs.msg import Pose, PoseStamped
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    BoundingVolume,
    Constraints,
    PositionConstraint,
)
from rclpy.action import ActionClient
from rclpy.node import Node
from shape_msgs.msg import SolidPrimitive


class TargetTask(Node):
    def __init__(self):
        super().__init__("target_task")

        self.client = ActionClient(self, MoveGroup, "/move_action")
        self.sub = self.create_subscription(
            PoseStamped,
            "/vision/target_pose",
            self.on_target,
            10,
        )

        self.busy = False
        self.done = False

        self.get_logger().info("Waiting for /move_action...")
        self.client.wait_for_server()
        self.get_logger().info("MoveGroup action is ready")

    def on_target(self, msg):
        if self.busy or self.done:
            return

        self.busy = True

        goal = MoveGroup.Goal()
        goal.request.group_name = "panda_arm"
        goal.request.num_planning_attempts = 10
        goal.request.allowed_planning_time = 5.0
        goal.request.max_velocity_scaling_factor = 0.2
        goal.request.max_acceleration_scaling_factor = 0.2
        goal.request.start_state.is_diff = True

        position_constraint = PositionConstraint()
        position_constraint.header.frame_id = "panda_link0"
        position_constraint.header.stamp = msg.header.stamp
        position_constraint.link_name = "panda_link8"
        position_constraint.target_point_offset.x = 0.0
        position_constraint.target_point_offset.y = 0.0
        position_constraint.target_point_offset.z = 0.0

        region = BoundingVolume()
        sphere = SolidPrimitive()
        sphere.type = SolidPrimitive.SPHERE
        sphere.dimensions = [0.03]
        region.primitives.append(sphere)

        target_pose = Pose()
        target_pose.position.x = msg.pose.position.x
        target_pose.position.y = msg.pose.position.y
        target_pose.position.z = msg.pose.position.z
        target_pose.orientation.w = 1.0
        region.primitive_poses.append(target_pose)

        position_constraint.constraint_region = region
        position_constraint.weight = 1.0

        constraints = Constraints()
        constraints.position_constraints.append(position_constraint)
        goal.request.goal_constraints.append(constraints)

        goal.planning_options.plan_only = False

        self.get_logger().info(
            f"Sending target: "
            f"x={target_pose.position.x:.3f}, "
            f"y={target_pose.position.y:.3f}, "
            f"z={target_pose.position.z:.3f}"
        )

        future = self.client.send_goal_async(goal)
        future.add_done_callback(self.on_goal_response)

    def on_goal_response(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn("MoveGroup goal rejected")
            self.busy = False
            self.done = True
            return

        self.get_logger().info("MoveGroup goal accepted")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.on_result)

    def on_result(self, future):
        result = future.result().result
        code = result.error_code.val
        self.done = True
        self.busy = False

        if code == 1:
            self.get_logger().info("MoveGroup execution succeeded")
        else:
            self.get_logger().warn(
                f"MoveGroup execution failed, error_code={code}"
            )


def main(args=None):
    rclpy.init(args=args)
    node = TargetTask()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
