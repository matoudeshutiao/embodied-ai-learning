#!/usr/bin/env bash
set -euo pipefail

# 先启动：
# ros2 launch vla_bridge real_controller_demo.launch.py \
#   start_mock_policy:=false max_in_flight:=2 max_action_age:=0.5 \
#   state_timeout:=0.5 watchdog_timeout:=60.0
#
# 等 safe_executor 打印 "Trajectory action server ready" 后，再运行本脚本。

source /opt/ros/humble/setup.bash
source "$HOME/ros2_ws/install/setup.bash"

echo "[1/5] 发送 30 秒长轨迹（脚本自己计时，不需要你抢操作）"
ros2 topic pub --once /policy/action_raw \
  trajectory_msgs/msg/JointTrajectory \
  "{joint_names: [joint1, joint2], points: [{positions: [0.5, -0.3], time_from_start: {sec: 30, nanosec: 0}}]}"

echo "[2/5] 等待 5 秒，让轨迹进入 Active"
sleep 5

echo "[3/5] 停掉 joint_state_broadcaster，触发状态超时"
ros2 control switch_controllers --deactivate joint_state_broadcaster

echo "[4/5] 等待 5 秒，观察安全层取消 active 轨迹"
sleep 5

echo "[5/5] 恢复 broadcaster，然后发送一次 queue_probe 测试恢复"
ros2 control switch_controllers --activate joint_state_broadcaster
sleep 3
timeout 10 ros2 run vla_bridge queue_probe --ros-args -p use_sim_time:=true || true

echo "测试结束。检查 launch 终端里的："
echo "  SAFETY STOP: state stale"
echo "  cancel active trajectory"
echo "  joint state restored"
echo "  safety recovered"
