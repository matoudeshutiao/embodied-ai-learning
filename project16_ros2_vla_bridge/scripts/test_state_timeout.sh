#!/usr/bin/env bash
# 状态超时 → 中断 → 恢复 的隔离测试。
#
# 这个脚本故意不使用 queue_probe：
# queue_probe 是上一关的队列压力测试，不适合混进“状态中断/恢复”实验。
#
# 用法：
#   正常执行：bash test_state_timeout.sh
#   只检查命令顺序：DRY_RUN=1 bash test_state_timeout.sh
set -eo pipefail
set +u

DRY_RUN="${DRY_RUN:-0}"

run() {
  printf '+'
  printf ' %q' "$@"
  printf '\n'
  if [ "$DRY_RUN" != "1" ]; then
    "$@"
  fi
}

wait_seconds() {
  echo "[wait] $1 s"
  if [ "$DRY_RUN" != "1" ]; then
    sleep "$1"
  fi
}

if [ "$DRY_RUN" != "1" ]; then
  if [ ! -f /opt/ros/humble/setup.bash ]; then
    echo "ERROR: /opt/ros/humble/setup.bash not found"
    exit 1
  fi

  if [ ! -f "$HOME/ros2_ws/install/setup.bash" ]; then
    echo "ERROR: ~/ros2_ws/install/setup.bash not found"
    exit 1
  fi

  # ROS setup.bash 内部可能引用未定义变量，不要在它前面启用 set -u。
  source /opt/ros/humble/setup.bash
  source "$HOME/ros2_ws/install/setup.bash"

  if ! command -v ros2 >/dev/null 2>&1; then
    echo "ERROR: ros2 command not found after sourcing"
    exit 1
  fi

  if ! ros2 action list | grep -q \
    "/joint_trajectory_controller/follow_joint_trajectory"; then
    echo "ERROR: controller action not found; start the launch first."
    exit 1
  fi
fi

echo "[1/4] 发送一条慢速 20 秒轨迹，只测中断，不测队列"
run ros2 topic pub --once /policy/action_raw \
  trajectory_msgs/msg/JointTrajectory \
  "{joint_names: [joint1, joint2], points: [{positions: [0.2, -0.1], time_from_start: {sec: 20, nanosec: 0}}]}"

wait_seconds 5

echo "[2/4] 停掉 joint_state_broadcaster，触发状态超时"
run ros2 control switch_controllers --deactivate joint_state_broadcaster

wait_seconds 5

echo "[3/4] 恢复 broadcaster"
run ros2 control switch_controllers --activate joint_state_broadcaster

wait_seconds 3

echo "[4/4] 发送一条短恢复轨迹，验证恢复后还能执行"
run ros2 topic pub --once /policy/action_raw \
  trajectory_msgs/msg/JointTrajectory \
  "{joint_names: [joint1, joint2], points: [{positions: [0.0, 0.0], time_from_start: {sec: 2, nanosec: 0}}]}"

wait_seconds 3

echo "测试结束。检查 launch 终端："
echo "  SAFETY STOP: state stale"
echo "  cancel active trajectory"
echo "  joint state restored"
echo "  safety recovered, accepting actions again"
