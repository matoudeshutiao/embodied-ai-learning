#!/usr/bin/env python3
"""最小抓取流程：开夹爪 → pre-grasp → 下探 → 闭合 → 抬起。"""

import time

import rclpy
from builtin_interfaces.msg import Duration, Time
from control_msgs.action import FollowJointTrajectory
from gazebo_msgs.srv import GetModelState
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
from sensor_msgs.msg import JointState
from shape_msgs.msg import SolidPrimitive
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


FINGERS = ["panda_finger_joint1", "panda_finger_joint2"]

# panda_hand_joint 固定 yaw=-45°，这里用 +45° 补偿。
GRASP_QX = 0.9238795325
GRASP_QY = 0.3826834324


class PandaPick(Node):
    def __init__(self):
        super().__init__("panda_pick")
        self.target = None
        self.finger_positions = {}
        self.scene_pub = self.create_publisher(
            PlanningScene, "/planning_scene", 10
        )
        self.move_client = ActionClient(self, MoveGroup, "/move_action")
        self.model_client = self.create_client(
            GetModelState, "/gazebo/get_model_state"
        )
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
        self.create_subscription(
            JointState,
            "/joint_states",
            self.on_joint_state,
            20,
        )

    def on_joint_state(self, msg):
        values = dict(zip(msg.name, msg.position))
        for name in FINGERS:
            if name in values:
                self.finger_positions[name] = values[name]

    def settle(self, seconds=0.5):
        end = time.monotonic() + seconds
        while rclpy.ok() and time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.1)

    def get_cube_z(self):
        if not self.model_client.service_is_ready():
            if not self.model_client.wait_for_service(timeout_sec=2.0):
                self.get_logger().warn(
                    "/gazebo/get_model_state not available"
                )
                return None

        request = GetModelState.Request()
        request.model_name = "red_cube"
        request.relative_entity_name = "world"
        future = self.model_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
        if future.result() is None:
            self.get_logger().warn("get_model_state failed")
            return None
        return float(future.result().pose.position.z)

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
        orientation.orientation.x = GRASP_QX
        orientation.orientation.y = GRASP_QY
        orientation.orientation.w = 0.0
        orientation.absolute_x_axis_tolerance = 0.1
        orientation.absolute_y_axis_tolerance = 0.1
        orientation.absolute_z_axis_tolerance = 0.1
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
        before_z = self.get_cube_z()
        self.get_logger().info(
            f"cube z before pick: {before_z}"
        )

        self.remove_red_cube()
        for _ in range(5):
            rclpy.spin_once(self, timeout_sec=0.1)

        if not self.gripper(0.04, "open"):
            return
        if not self.move_to(x, y, z + 0.20, "pre-grasp"):
            return
        if not self.move_to(x, y, z + 0.075, "grasp"):
            return
        if not self.gripper(0.0, "close"):
            return

        self.settle(0.5)
        fingers = [
            self.finger_positions.get(name) for name in FINGERS
        ]
        finger_gap = (
            None
            if any(value is None for value in fingers)
            else sum(fingers) / len(fingers)
        )
        self.get_logger().info(
            f"finger positions after close: {fingers}, "
            f"average={finger_gap}"
        )

        if not self.move_to(x, y, z + 0.30, "lift"):
            return
        self.settle(0.5)
        after_z = self.get_cube_z()
        self.get_logger().info(
            f"cube z after lift: {after_z}"
        )

        finger_ok = finger_gap is not None and finger_gap > 0.005
        lift_ok = (
            before_z is not None
            and after_z is not None
            and (after_z - before_z) > 0.05
        )
        if finger_ok and lift_ok:
            self.get_logger().info("GRASP SUCCESS")
        else:
            self.get_logger().error(
                "GRASP FAILED: "
                f"finger_gap={finger_gap}, "
                f"cube_dz={None if after_z is None or before_z is None else after_z - before_z}"
            )
            self.gripper(0.04, "open")


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
