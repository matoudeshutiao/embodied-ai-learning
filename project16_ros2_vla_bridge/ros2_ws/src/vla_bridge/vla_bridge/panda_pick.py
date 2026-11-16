#!/usr/bin/env python3
"""最小抓取流程：开夹爪 → pre-grasp → 下探 → 闭合 → 抬起。"""

import rclpy
from builtin_interfaces.msg import Duration, Time
from control_msgs.action import FollowJointTrajectory
from geometry_msgs.msg import Pose, PoseStamped
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    BoundingVolume,
    CollisionObject,
    Constraints,
    MoveItErrorCodes,
    OrientationConstraint,
    PlanningScene,
    PositionConstraint,
)
from rclpy.action import ActionClient
from rclpy.node import Node
from shape_msgs.msg import SolidPrimitive
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


FINGERS = ["panda_finger_joint1", "panda_finger_joint2"]


class PandaPick(Node):
    def __init__(self):
        super().__init__("panda_pick")
        self.target = None
        self.scene_pub = self.create_publisher(
            PlanningScene, "/planning_scene", 10
        )
        self.move_client = ActionClient(self, MoveGroup, "/move_action")
        self.gripper_client = ActionClient(
            self,
            FollowJointTrajectory,
            "/panda_gripper_controller/follow_joint_trajectory",
        )
        self.create_subscription(
            PoseStamped,
            "/vision/target_pose",
            self.on_target,
            10,
        )

    def on_target(self, msg):
        if self.target is None:
            self.target = msg
            self.get_logger().info(
                "target received: "
                f"x={msg.pose.position.x:.3f}, "
                f"y={msg.pose.position.y:.3f}, "
                f"z={msg.pose.position.z:.3f}"
            )

    def remove_red_cube(self):
        scene = PlanningScene()
        scene.is_diff = True
        scene.robot_state.is_diff = True
        obj = CollisionObject()
        obj.header.frame_id = "panda_link0"
        obj.id = "red_cube"
        obj.operation = CollisionObject.REMOVE
        scene.world.collision_objects = [obj]
        self.scene_pub.publish(scene)
        self.get_logger().info("removed red_cube from planning scene")

    def move_to(self, x, y, z, label):
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
        pose.position.x = x
        pose.position.y = y
        pose.position.z = z
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
        constraints.name = label

        goal = MoveGroup.Goal()
        goal.request.group_name = "panda_arm"
        goal.request.num_planning_attempts = 10
        goal.request.allowed_planning_time = 5.0
        goal.request.max_velocity_scaling_factor = 0.15
        goal.request.max_acceleration_scaling_factor = 0.15
        goal.request.goal_constraints = [constraints]
        goal.planning_options.plan_only = False
        goal.planning_options.replan = True
        goal.planning_options.replan_attempts = 3

        self.get_logger().info(
            f"{label}: x={x:.3f}, y={y:.3f}, z={z:.3f}"
        )
        send_future = self.move_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future)
        handle = send_future.result()
        if handle is None or not handle.accepted:
            self.get_logger().error(f"{label}: goal rejected")
            return False

        result_future = handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result().result
        ok = result.error_code.val == MoveItErrorCodes.SUCCESS
        self.get_logger().info(
            f"{label}: code={result.error_code.val}, ok={ok}"
        )
        return ok

    def gripper(self, position, label):
        trajectory = JointTrajectory()
        trajectory.header.stamp = Time()
        trajectory.joint_names = FINGERS

        point = JointTrajectoryPoint()
        point.positions = [position, position]
        point.time_from_start = Duration(sec=1, nanosec=0)
        trajectory.points.append(point)

        goal = FollowJointTrajectory.Goal()
        goal.trajectory = trajectory

        self.get_logger().info(
            f"gripper {label}: position={position:.3f}"
        )
        send_future = self.gripper_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future)
        handle = send_future.result()
        if handle is None or not handle.accepted:
            self.get_logger().error(f"gripper {label}: rejected")
            return False

        result_future = handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result().result
        ok = result.error_code == 0
        self.get_logger().info(
            f"gripper {label}: code={result.error_code}, ok={ok}"
        )
        return ok

    def run(self):
        if not self.move_client.wait_for_server(timeout_sec=30.0):
            self.get_logger().error("/move_action not ready")
            return
        if not self.gripper_client.wait_for_server(timeout_sec=30.0):
            self.get_logger().error("gripper action not ready")
            return

        while rclpy.ok() and self.target is None:
            rclpy.spin_once(self, timeout_sec=0.1)
        if self.target is None:
            return

        x = self.target.pose.position.x
        y = self.target.pose.position.y
        z = self.target.pose.position.z

        self.remove_red_cube()
        for _ in range(5):
            rclpy.spin_once(self, timeout_sec=0.1)

        if not self.gripper(0.04, "open"):
            return
        if not self.move_to(x, y, z + 0.20, "pre-grasp"):
            return
        if not self.move_to(x, y, z + 0.10, "grasp"):
            return
        if not self.gripper(0.0, "close"):
            return
        if not self.move_to(x, y, z + 0.30, "lift"):
            return
        self.get_logger().info("pick sequence finished")


def main(args=None):
    rclpy.init(args=args)
    node = PandaPick()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
