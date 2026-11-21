#!/usr/bin/env python3
"""A3：随机位置抓放成功率试验。

前置条件：
  1. 已启动 vla_bridge/panda_moveit_gazebo.launch.py
  2. 运行本脚本的终端已经 source ROS 和工作空间

用法：
  python3 project16_ros2_vla_bridge/scripts/panda_pick_trials.py --trials 3
  python3 project16_ros2_vla_bridge/scripts/panda_pick_trials.py --trials 20 --seed 0

每次试验：
  detach 残留关节 -> 张开夹爪 -> /gazebo/set_entity_state 传送方块
  -> 重发 planning scene 的 red_cube -> panda_home 复位
  -> 运行 panda_pick -> 从 /model_states 独立校验 -> 写 CSV
"""

import argparse
import csv
import math
import os
import random
import re
import signal
import subprocess
import sys
import threading
import time
from datetime import datetime

import rclpy
from gazebo_msgs.msg import ModelStates
from gazebo_msgs.srv import SetEntityState
from geometry_msgs.msg import Pose
from linkattacher_msgs.srv import DetachLink
from moveit_msgs.msg import CollisionObject, PlanningScene
from rclpy.node import Node
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import Float64MultiArray

CUBE_NAME = "red_cube"
CUBE_SIZE = 0.05
CUBE_Z = 0.375
PLACE_TOL_XY = 0.02
PLACE_TOL_Z = 0.03


class TrialRunner(Node):
    def __init__(self, args):
        super().__init__("a3_trial_runner")
        self.args = args
        self.cube_pose = None
        self.cube_stamp = 0.0

        self.create_subscription(
            ModelStates, "/model_states", self.on_model_states, 10
        )
        self.grip_pub = self.create_publisher(
            Float64MultiArray,
            "/panda_gripper_effort_controller/commands",
            10,
        )
        self.scene_pub = self.create_publisher(
            PlanningScene, "/planning_scene", 10
        )

        self.set_state_client = self.find_service(
            SetEntityState,
            ("/set_entity_state", "/gazebo/set_entity_state"),
            5.0,
        )
        if self.set_state_client is None:
            raise RuntimeError(
                "找不到 /set_entity_state 或 /gazebo/set_entity_state；"
                "先确认 Gazebo 里 gazebo_ros_state 插件已加载"
            )

        self.detach_client = self.create_client(DetachLink, "/DETACHLINK")

    def find_service(self, srv_type, names, timeout):
        for name in names:
            client = self.create_client(srv_type, name)
            if client.wait_for_service(timeout_sec=timeout):
                self.get_logger().info(f"using service {name}")
                return client
        return None

    def on_model_states(self, msg):
        if CUBE_NAME not in msg.name:
            return
        index = msg.name.index(CUBE_NAME)
        position = msg.pose[index].position
        self.cube_pose = (
            float(position.x),
            float(position.y),
            float(position.z),
        )
        self.cube_stamp = time.monotonic()

    def spin_for(self, seconds):
        end = time.monotonic() + seconds
        while rclpy.ok() and time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.05)

    def read_cube_pose(self, timeout=2.0):
        self.cube_pose = None
        end = time.monotonic() + timeout
        while (
            rclpy.ok()
            and self.cube_pose is None
            and time.monotonic() < end
        ):
            rclpy.spin_once(self, timeout_sec=0.05)
        return self.cube_pose

    def open_gripper(self):
        msg = Float64MultiArray()
        msg.data = [10.0, 10.0]
        end = time.monotonic() + 0.5
        while rclpy.ok() and time.monotonic() < end:
            self.grip_pub.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.02)
            time.sleep(0.02)

    def detach_residual(self):
        """上一轮如果还挂着固定关节，先解开。没有关节也算成功。"""
        if not self.detach_client.wait_for_service(timeout_sec=2.0):
            return False, "/DETACHLINK 服务不存在"

        messages = []
        for link_name in ("panda_link7", "panda_link8", "panda_hand"):
            request = DetachLink.Request()
            request.model1_name = "panda"
            request.link1_name = link_name
            request.model2_name = CUBE_NAME
            request.link2_name = "link"
            future = self.detach_client.call_async(request)
            rclpy.spin_until_future_complete(
                self, future, timeout_sec=3.0
            )
            if not future.done():
                return False, f"detach {link_name} 超时"
            response = future.result()
            if response is None:
                return False, f"detach {link_name} 无响应"
            message = response.message or ""
            messages.append(f"{link_name}: {message}")
            if response.success:
                return True, message
            if "does not exist" not in message:
                return False, message
        return True, "no residual joint"

    def teleport_cube(self, x, y, z):
        request = SetEntityState.Request()
        request.state.name = CUBE_NAME
        request.state.pose.position.x = x
        request.state.pose.position.y = y
        request.state.pose.position.z = z
        request.state.pose.orientation.w = 1.0
        request.state.twist.linear.x = 0.0
        request.state.twist.linear.y = 0.0
        request.state.twist.linear.z = 0.0
        request.state.twist.angular.x = 0.0
        request.state.twist.angular.y = 0.0
        request.state.twist.angular.z = 0.0
        request.state.reference_frame = "world"

        future = self.set_state_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        if not future.done():
            return False, "set_entity_state 超时"
        response = future.result()
        if response is None:
            return False, "set_entity_state 无响应"
        # Humble 的 SetEntityState 响应只有 success，没有 status_message。
        message = getattr(response, "status_message", "")
        if not message and not response.success:
            message = "set_entity_state returned success=False"
        return bool(response.success), message

    def publish_cube_scene(self, x, y, z):
        scene = PlanningScene()
        scene.is_diff = True
        scene.robot_state.is_diff = True

        obj = CollisionObject()
        obj.header.frame_id = "panda_link0"
        obj.id = CUBE_NAME
        obj.operation = CollisionObject.ADD

        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.BOX
        primitive.dimensions = [CUBE_SIZE, CUBE_SIZE, CUBE_SIZE]

        pose = Pose()
        pose.position.x = x
        pose.position.y = y
        pose.position.z = z
        pose.orientation.w = 1.0

        obj.primitives = [primitive]
        obj.primitive_poses = [pose]
        scene.world.collision_objects = [obj]

        for _ in range(3):
            self.scene_pub.publish(scene)
            rclpy.spin_once(self, timeout_sec=0.05)
            time.sleep(0.1)

    def run_home(self):
        env = os.environ.copy()
        env["PYTHONUNBUFFERED"] = "1"
        try:
            proc = subprocess.run(
                ["ros2", "run", "vla_bridge", "panda_home"],
                capture_output=True,
                text=True,
                timeout=40.0,
                env=env,
            )
        except subprocess.TimeoutExpired:
            return False, "panda_home 超时"
        output = (proc.stdout or "") + (proc.stderr or "")
        return "home result: code=0" in output, output

    def run_pick(self, log_path):
        command = [
            "ros2",
            "run",
            "vla_bridge",
            "panda_pick",
            "--ros-args",
            "-p",
            f"place_x:={self.args.place_x}",
            "-p",
            f"place_y:={self.args.place_y}",
            "-p",
            f"place_z:={self.args.place_z}",
        ]
        env = os.environ.copy()
        env["PYTHONUNBUFFERED"] = "1"
        state = {"timed_out": False}
        lines = []

        with open(log_path, "w", encoding="utf-8") as log_file:
            proc = subprocess.Popen(
                command,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                bufsize=1,
                env=env,
                start_new_session=True,
            )

            def kill_on_timeout():
                state["timed_out"] = True
                try:
                    os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
                except Exception:
                    proc.kill()

            timer = threading.Timer(self.args.timeout, kill_on_timeout)
            timer.start()
            try:
                key_markers = (
                    "GRASP SUCCESS",
                    "GRASP FAILED",
                    "EMPTY GRASP",
                    "GRASP ABORTED",
                    "PLACE SUCCESS",
                    "PLACE FAILED",
                    "PLACE ABORTED",
                    "vision vs truth",
                    "pre-grasp: code",
                    "attach link",
                    "detach link",
                    "place error",
                    "service timeout",
                    "no /vision",
                    "fingers stable",
                )
                for line in proc.stdout:
                    lines.append(line.rstrip("\n"))
                    log_file.write(line)
                    if any(marker in line for marker in key_markers):
                        print(f"    {line}", end="")
                proc.wait()
            finally:
                timer.cancel()

        return lines, state["timed_out"]

    @staticmethod
    def classify(lines, timed_out, home_ok):
        text = "\n".join(lines)
        result = {
            "grasp_ok": "GRASP SUCCESS" in text,
            "attach_ok": "attach link panda_link7: success=True" in text,
            "place_ok": "PLACE SUCCESS" in text,
        }
        match = re.search(r"err_xy=([0-9.]+) mm", text)
        result["vision_err_mm"] = (
            float(match.group(1)) if match else None
        )

        if not home_ok:
            stage = "home"
        elif timed_out or "service timeout" in text:
            stage = "system"
        elif (
            "no /vision/target_pose" in text
            or "vision samples" in text
        ):
            stage = "vision"
        elif "pre-grasp: code=1, ok=True" not in text:
            stage = "plan"
        elif "EMPTY GRASP" in text or "GRASP FAILED" in text:
            stage = "grasp"
        elif "GRASP ABORTED" in text:
            stage = "attach"
        elif "PLACE FAILED" in text or "PLACE ABORTED" in text:
            stage = "place"
        elif not result["place_ok"]:
            stage = "unknown"
        else:
            stage = "ok"
        result["stage"] = stage
        return result

    def run_trials(self):
        args = self.args
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        out_dir = os.path.abspath(
            os.path.join(args.out_dir, f"a3_{stamp}")
        )
        os.makedirs(out_dir, exist_ok=True)
        csv_path = os.path.join(out_dir, "summary.csv")

        if self.read_cube_pose(timeout=5.0) is None:
            raise RuntimeError(
                "5 秒内没有收到 /model_states；确认 Gazebo 已启动"
            )

        fieldnames = [
            "trial",
            "cube_x",
            "cube_y",
            "vision_err_mm",
            "grasp_ok",
            "attach_ok",
            "place_ok_node",
            "cube_after_x",
            "cube_after_y",
            "cube_after_z",
            "place_err_xy_mm",
            "place_err_z_mm",
            "place_verified",
            "full_success",
            "failure_stage",
            "home_ok",
            "timed_out",
            "log_file",
        ]

        rng = random.Random(args.seed)
        rows = []
        print(f"results: {out_dir}")

        with open(csv_path, "w", newline="", encoding="utf-8") as handle:
            writer = csv.DictWriter(handle, fieldnames=fieldnames)
            writer.writeheader()

            for trial in range(1, args.trials + 1):
                x = round(rng.uniform(args.x_min, args.x_max), 4)
                y = round(rng.uniform(args.y_min, args.y_max), 4)
                print(
                    f"\n=== trial {trial}/{args.trials}: "
                    f"cube=({x:.4f}, {y:.4f}, {CUBE_Z}) ==="
                )

                detach_ok, detach_message = self.detach_residual()
                if not detach_ok:
                    print(f"ABORT: detach failed: {detach_message}")
                    break

                self.open_gripper()

                teleport_ok, teleport_message = self.teleport_cube(
                    x, y, CUBE_Z
                )
                if not teleport_ok:
                    print(
                        f"ABORT: teleport failed: {teleport_message}; "
                        "Gazebo 可能已经崩溃"
                    )
                    break

                self.publish_cube_scene(x, y, CUBE_Z)
                self.spin_for(0.8)

                home_ok, home_output = self.run_home()
                if not home_ok:
                    print("ABORT: panda_home failed")
                    print(home_output)
                    break
                self.spin_for(1.0)

                log_path = os.path.join(
                    out_dir, f"trial_{trial:02d}.log"
                )
                lines, timed_out = self.run_pick(log_path)
                self.spin_for(0.5)

                after = self.read_cube_pose(timeout=2.0)
                if after is None:
                    after_err_xy = None
                    after_err_z = None
                    verified = False
                else:
                    after_err_xy = math.hypot(
                        after[0] - args.place_x,
                        after[1] - args.place_y,
                    )
                    after_err_z = abs(after[2] - args.place_z)
                    verified = (
                        after_err_xy < PLACE_TOL_XY
                        and after_err_z < PLACE_TOL_Z
                    )

                info = self.classify(lines, timed_out, True)
                full_success = (
                    info["place_ok"] and verified and not timed_out
                )
                if info["place_ok"] and not verified:
                    print(
                        "WARNING: node reported PLACE SUCCESS, "
                        "independent pose check failed"
                    )
                if info["stage"] == "ok" and not full_success:
                    info["stage"] = "verify"

                row = {
                    "trial": trial,
                    "cube_x": x,
                    "cube_y": y,
                    "vision_err_mm": info["vision_err_mm"],
                    "grasp_ok": info["grasp_ok"],
                    "attach_ok": info["attach_ok"],
                    "place_ok_node": info["place_ok"],
                    "cube_after_x": None if after is None else after[0],
                    "cube_after_y": None if after is None else after[1],
                    "cube_after_z": None if after is None else after[2],
                    "place_err_xy_mm": (
                        None
                        if after_err_xy is None
                        else round(after_err_xy * 1000.0, 1)
                    ),
                    "place_err_z_mm": (
                        None
                        if after_err_z is None
                        else round(after_err_z * 1000.0, 1)
                    ),
                    "place_verified": verified,
                    "full_success": full_success,
                    "failure_stage": info["stage"],
                    "home_ok": home_ok,
                    "timed_out": timed_out,
                    "log_file": os.path.basename(log_path),
                }
                rows.append(row)
                writer.writerow(row)
                handle.flush()

                print(
                    f"trial {trial}: full={full_success} "
                    f"grasp={info['grasp_ok']} "
                    f"place={info['place_ok']} "
                    f"stage={info['stage']} "
                    f"vision_err={info['vision_err_mm']} mm "
                    f"place_err="
                    f"{row['place_err_xy_mm']}/{row['place_err_z_mm']} mm"
                )

                if timed_out:
                    print(
                        "ABORT: panda_pick 超时；"
                        "Gazebo 可能已经崩溃，请整场重启"
                    )
                    break

        self.print_summary(rows, out_dir, csv_path)

    @staticmethod
    def print_summary(rows, out_dir, csv_path):
        total = len(rows)
        if total == 0:
            print("没有完成任何试验")
            return
        full = sum(1 for row in rows if row["full_success"])
        grasp = sum(1 for row in rows if row["grasp_ok"])
        stages = {}
        for row in rows:
            stages[row["failure_stage"]] = (
                stages.get(row["failure_stage"], 0) + 1
            )
        vision_values = [
            row["vision_err_mm"]
            for row in rows
            if row["vision_err_mm"] is not None
        ]
        place_values = [
            row["place_err_xy_mm"]
            for row in rows
            if row["place_err_xy_mm"] is not None
            and row["place_verified"]
        ]

        print("\n===== A3 SUMMARY =====")
        print(f"trials: {total}")
        print(
            f"full success: {full}/{total} "
            f"({100.0 * full / total:.1f}%)"
        )
        print(
            f"grasp success: {grasp}/{total} "
            f"({100.0 * grasp / total:.1f}%)"
        )
        for stage, count in sorted(stages.items()):
            print(f"  {stage}: {count}")
        if vision_values:
            print(
                f"vision error: mean={sum(vision_values) / len(vision_values):.1f} mm, "
                f"max={max(vision_values):.1f} mm"
            )
        if place_values:
            print(
                f"place error xy: mean={sum(place_values) / len(place_values):.1f} mm, "
                f"max={max(place_values):.1f} mm"
            )
        print(f"csv: {csv_path}")
        print(f"logs: {out_dir}")


def parse_args(argv):
    parser = argparse.ArgumentParser(
        description="A3 random-position pick-and-place trials"
    )
    parser.add_argument("--trials", type=int, default=3)
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--x-min", type=float, default=0.42)
    parser.add_argument("--x-max", type=float, default=0.48)
    parser.add_argument("--y-min", type=float, default=-0.18)
    parser.add_argument("--y-max", type=float, default=-0.12)
    parser.add_argument("--place-x", type=float, default=0.40)
    parser.add_argument("--place-y", type=float, default=0.08)
    parser.add_argument("--place-z", type=float, default=0.375)
    parser.add_argument("--timeout", type=float, default=180.0)
    parser.add_argument(
        "--out-dir",
        default=os.path.join(
            os.path.dirname(__file__),
            "..",
            "output",
        ),
    )
    return parser.parse_args(argv)


def main(argv=None):
    args = parse_args(argv)
    rclpy.init()
    try:
        runner = TrialRunner(args)
    except Exception as exc:
        print(f"ERROR: {exc}")
        rclpy.try_shutdown()
        return 1
    try:
        runner.run_trials()
    except KeyboardInterrupt:
        pass
    finally:
        runner.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
