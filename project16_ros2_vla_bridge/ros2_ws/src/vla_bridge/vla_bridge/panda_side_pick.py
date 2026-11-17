#!/usr/bin/env python3
"""侧向抓取：水平夹爪先到方块上方，再下探从两侧夹住。"""

import time

import rclpy
from builtin_interfaces.msg import Duration, Time as MsgTime
from control_msgs.action import FollowJointTrajectory
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
from tf2_ros import Buffer, TransformListener
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


FINGERS = ["panda_finger_joint1", "panda_finger_joint2"]
SIDE_QY = 0.7071067811865476


class PandaSidePick(Node):
    def __init__(self):
        super().__init__("panda_side_pick")
        self.target = None
        self.cube_pose = None
        self.finger_positions = {}

        self.declare_parameter("grasp_x_offset", 0.10)
        self.declare_parameter("side_z_offset", 0.02)
        self.declare_parameter("pre_z_offset", 0.12)
        self.declare_parameter("lift_height", 0.20)
        self.declare_parameter("close_position", 0.02)
        self.grasp_x_offset = float(
            self.get_parameter("grasp_x_offset").value
        )
        self.side_z_offset = float(
            self.get_parameter("side_z_offset").value
        )
        self.pre_z_offset = float(
            self.get_parameter("pre_z_offset").value
        )
        self.lift_height = float(
            self.get_parameter("lift_height").value
        )
        self.close_position = float(
            self.get_parameter("close_position").value
        )

        self.scene_pub = self.create_publisher(
            PlanningScene, "/planning_scene", 10
        )
        self.move_client = ActionClient(self, MoveGroup, "/move_action")
        self.gripper_client = ActionClient(
            self,
            FollowJointTrajectory,
            "/panda_gripper_controller/follow_joint_trajectory",
        )
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.create_subscription(
            PoseStamped, "/vision/target_pose", self.on_target, 10
        )
        self.create_subscription(
            JointState, "/joint_states", self.on_joint_state, 20
        )
        self.create_subscription(
            ModelStates, "/model_states", self.on_model_states, 10
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

    def on_joint_state(self, msg):
        values = dict(zip(msg.name, msg.position))
        for name in FINGERS:
            if name in values:
                self.finger_positions[name] = values[name]

    def on_model_states(self, msg):
        if "red_cube" in msg.name:
            index = msg.name.index("red_cube")
            p = msg.pose[index].position
            self.cube_pose = (float(p.x), float(p.y), float(p.z))

    def get_cube_pose(self):
        self.cube_pose = None
        deadline = time.monotonic() + 2.0
        while rclpy.ok() and self.cube_pose is None:
            if time.monotonic() > deadline:
                self.get_logger().warn("/model_states has no red_cube")
                return None
            rclpy.spin_once(self, timeout_sec=0.1)
        return self.cube_pose

    def settle(self, seconds=0.5):
        end = time.monotonic() + seconds
        while rclpy.ok() and time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.1)

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
        position = PositionConstraint()
        position.header.frame_id = "panda_link0"
        position.link_name = "panda_link8"
        position.weight = 1.0

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
        position.constraint_region = region

        # 侧向抓取：hand 的 z 轴朝 +x，手指沿 world y 闭合。
        orientation = OrientationConstraint()
        orientation.header.frame_id = "panda_link0"
        orientation.link_name = "panda_hand"
        orientation.orientation.y = SIDE_QY
        orientation.orientation.w = SIDE_QY
        orientation.absolute_x_axis_tolerance = 0.1
        orientation.absolute_y_axis_tolerance = 0.1
        orientation.absolute_z_axis_tolerance = 0.1
        orientation.weight = 1.0

        constraints = Constraints()
        constraints.position_constraints = [position]
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
        trajectory.header.stamp = MsgTime()
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

        self.get_logger().info("waiting for /vision/target_pose")
        deadline = time.monotonic() + 10.0
        while rclpy.ok() and self.target is None:
            if time.monotonic() > deadline:
                self.get_logger().warn(
                    "no /vision/target_pose after 10s"
                )
                return
            rclpy.spin_once(self, timeout_sec=0.1)

        cube = self.get_cube_pose()
        if cube is None:
            return
        cx, cy, cz = cube
        self.get_logger().info(
            f"side grasp cube: x={cx:.3f}, y={cy:.3f}, z={cz:.3f}"
        )

        # 等 panda_scene 的 3 次发布结束，避免它把红方块重新加回来。
        self.settle(3.0)
        self.remove_red_cube()
        self.settle(0.3)

        if not self.gripper(0.04, "open"):
            return
        grasp_x = cx - self.grasp_x_offset
        z = cz + self.side_z_offset
        pre_z = z + self.pre_z_offset

        if not self.move_to(grasp_x, cy, pre_z, "side pre-grasp"):
            return
        if not self.move_to(grasp_x, cy, z, "side grasp"):
            return
        self.get_logger().info(
            f"cube before close: {self.get_cube_pose()}"
        )
        if not self.gripper(self.close_position, "close"):
            return
        self.settle(0.5)

        fingers = [
            self.finger_positions.get(name) for name in FINGERS
        ]
        finger_gap = (
            None
            if any(v is None for v in fingers)
            else sum(fingers) / len(fingers)
        )
        self.get_logger().info(
            f"finger positions after close: {fingers}, "
            f"average={finger_gap}"
        )
        self.get_logger().info(
            f"cube after close: {self.get_cube_pose()}"
        )

        before = self.get_cube_pose()
        if not self.move_to(grasp_x, cy, z + self.lift_height, "lift"):
            return
        self.settle(0.5)
        after = self.get_cube_pose()
        self.get_logger().info(f"cube after lift: {after}")

        finger_ok = finger_gap is not None and finger_gap > 0.005
        lift_ok = (
            before is not None
            and after is not None
            and (after[2] - before[2]) > 0.05
        )
        if finger_ok and lift_ok:
            self.get_logger().info("SIDE GRASP SUCCESS")
        else:
            self.get_logger().error(
                "SIDE GRASP FAILED: "
                f"finger_gap={finger_gap}, before={before}, after={after}"
            )
            self.gripper(0.04, "open")


def main(args=None):
    rclpy.init(args=args)
    node = PandaSidePick()
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
