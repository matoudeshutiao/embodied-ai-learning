#!/usr/bin/env python3
"""最小抓取流程：开夹爪 → pre-grasp → 下探 → 闭合 → 抬起。"""

import time

import rclpy
from gazebo_msgs.msg import ModelStates
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
from rclpy.time import Time as RosTime
from sensor_msgs.msg import JointState
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import Float64MultiArray
from tf2_ros import Buffer, TransformListener


FINGERS = ["panda_finger_joint1", "panda_finger_joint2"]

# panda_hand_joint 固定 yaw=-45°，这里用 +45° 补偿。
GRASP_QX = 0.9238795325
GRASP_QY = 0.3826834324


class PandaPick(Node):
    def __init__(self):
        super().__init__("panda_pick")
        self.target = None
        self.finger_positions = {}
        self.declare_parameter("pre_grasp_offset", 0.20)
        self.declare_parameter("grasp_offset", 0.095)
        self.declare_parameter("gripper_effort", 1.0)
        self.declare_parameter("fixed_cube_test", True)
        self.pre_grasp_offset = float(
            self.get_parameter("pre_grasp_offset").value
        )
        self.grasp_offset = float(
            self.get_parameter("grasp_offset").value
        )
        self.gripper_effort = float(
            self.get_parameter("gripper_effort").value
        )
        self.scene_pub = self.create_publisher(
            PlanningScene, "/planning_scene", 10
        )
        self.move_client = ActionClient(self, MoveGroup, "/move_action")
        self.effort_pub = self.create_publisher(
            Float64MultiArray,
            "/panda_gripper_effort_controller/commands",
            10,
        )
        self.cube_pose = None
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
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
        self.create_subscription(
            ModelStates,
            "/model_states",
            self.on_model_states,
            10,
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

    def on_model_states(self, msg):
        if "red_cube" in msg.name:
            index = msg.name.index("red_cube")
            pose = msg.pose[index].position
            self.cube_pose = (
                float(pose.x),
                float(pose.y),
                float(pose.z),
            )

    def get_cube_pose(self):
        self.cube_pose = None
        deadline = time.monotonic() + 2.0
        while rclpy.ok() and self.cube_pose is None:
            if time.monotonic() > deadline:
                self.get_logger().warn("/model_states has no red_cube")
                return None
            rclpy.spin_once(self, timeout_sec=0.1)
        return self.cube_pose

    def get_cube_z(self):
        pose = self.get_cube_pose()
        return None if pose is None else pose[2]

    def link_xyz(self, frame):
        try:
            transform = self.tf_buffer.lookup_transform(
                "panda_link0", frame, RosTime()
            )
            p = transform.transform.translation
            return (float(p.x), float(p.y), float(p.z))
        except Exception as exc:
            self.get_logger().warn(f"TF {frame} failed: {exc}")
            return None

    def on_target(self, msg):
        if self.target is None:
            self.target = msg
            self.get_logger().info(
                "target received: "
                f"x={msg.pose.position.x:.3f}, "
                f"y={msg.pose.position.y:.3f}, "
                f"z={msg.pose.position.z:.3f}"
            )

    def remove_collision(self, object_id):
        scene = PlanningScene()
        scene.is_diff = True
        scene.robot_state.is_diff = True
        obj = CollisionObject()
        obj.header.frame_id = "panda_link0"
        obj.id = object_id
        obj.operation = CollisionObject.REMOVE
        scene.world.collision_objects = [obj]
        self.scene_pub.publish(scene)
        self.get_logger().info(
            f"removed {object_id} from planning scene"
        )

    def remove_red_cube(self):
        self.remove_collision("red_cube")

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

    def effort(self, value, seconds, label):
        msg = Float64MultiArray()
        msg.data = [value, value]
        self.get_logger().info(
            f"gripper {label}: effort={value:.2f} Nm"
        )
        end = time.monotonic() + seconds
        while rclpy.ok() and time.monotonic() < end:
            self.effort_pub.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.05)
            time.sleep(0.05)

    def run(self):
        if not self.move_client.wait_for_server(timeout_sec=30.0):
            self.get_logger().error("/move_action not ready")
            return

        self.get_logger().info("waiting for /vision/target_pose")
        deadline = time.monotonic() + 10.0
        while rclpy.ok() and self.target is None:
            if time.monotonic() > deadline:
                self.get_logger().warn(
                    "no /vision/target_pose after 10s; "
                    "cube may be out of camera view"
                )
                return
            rclpy.spin_once(self, timeout_sec=0.1)
        if self.target is None:
            return

        # 用 Gazebo 里的实际方块位姿，不用视觉 z。
        cube = self.get_cube_pose()
        if cube is None:
            return
        x, y, z = cube
        before_z = z
        self.get_logger().info(
            f"vertical grasp cube: x={x:.3f}, y={y:.3f}, z={z:.3f}"
        )

        # 等 panda_scene 的 3 次发布结束，避免它重新加回红方块。
        self.settle(3.0)
        self.remove_red_cube()
        self.settle(0.3)

        self.effort(self.gripper_effort, 0.6, "open")
        if not self.move_to(x, y, z + self.pre_grasp_offset, "pre-grasp"):
            return
        if not self.move_to(x, y, z + self.grasp_offset, "grasp"):
            return
        cube_pose = self.get_cube_pose()
        self.get_logger().info(
            f"cube before close: {cube_pose}, "
            f"link8: {self.link_xyz('panda_link8')}, "
            f"leftfinger: {self.link_xyz('panda_leftfinger')}, "
            f"rightfinger: {self.link_xyz('panda_rightfinger')}"
        )
        fixed_cube_test = bool(
            self.get_parameter("fixed_cube_test").value
        )
        if fixed_cube_test:
            self.get_logger().info(
                "fixed cube test mode: effort close, lift skipped"
            )
        self.effort(-self.gripper_effort, 2.0, "close")

        self.settle(0.5)
        self.get_logger().info(
            f"cube after close: {self.get_cube_pose()}"
        )
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

        if fixed_cube_test:
            # 手指宽度 0.02097、方块宽 0.05，理论接触位置约 0.0355。
            contact_ok = all(
                value is not None and value > 0.030
                for value in fingers
            )
            if contact_ok:
                self.get_logger().info(
                    f"FIXED CUBE CONTACT OK: gap={finger_gap:.4f}"
                )
            else:
                self.get_logger().error(
                    f"FIXED CUBE CONTACT FAILED: gap={finger_gap}"
                )
            self.effort(self.gripper_effort, 0.6, "open")
            return

        if not self.move_to(x, y, z + 0.30, "lift"):
            return
        self.settle(0.5)
        after_pose = self.get_cube_pose()
        after_z = None if after_pose is None else after_pose[2]
        self.get_logger().info(f"cube after lift pose: {after_pose}")
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
            self.effort(self.gripper_effort, 0.6, "open")


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
