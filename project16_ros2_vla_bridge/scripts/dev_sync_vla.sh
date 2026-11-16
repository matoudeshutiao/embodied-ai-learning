#!/usr/bin/env bash
# 用法（必须 source，不能直接 bash 执行）：
#   source ~/embodied-ai-learning/project16_ros2_vla_bridge/scripts/dev_sync_vla.sh
#
# 作用：
#   git pull
#   同步 vla_bridge 到 ~/ros2_ws/src/vla_bridge
#   source ROS2
#   colcon build --packages-select vla_bridge
#   source 工作空间，保留在当前终端

set +u

if [ "${BASH_SOURCE[0]}" = "$0" ]; then
  echo "ERROR: 请用 source 运行，不要用 bash 运行。"
  echo "source ~/embodied-ai-learning/project16_ros2_vla_bridge/scripts/dev_sync_vla.sh"
  exit 1
fi

REPO="$HOME/embodied-ai-learning"
WS="$HOME/ros2_ws"
SRC="$REPO/project16_ros2_vla_bridge/ros2_ws/src/vla_bridge"
DST="$WS/src/vla_bridge"

for path in "$REPO" "$WS" "$SRC" "$DST"; do
  if [ ! -d "$path" ]; then
    echo "ERROR: 目录不存在: $path"
    return 1
  fi
done

if [ ! -f /opt/ros/humble/setup.bash ]; then
  echo "ERROR: 找不到 /opt/ros/humble/setup.bash"
  return 1
fi

cd "$REPO" || return 1

echo "== git pull =="
if [ "${SKIP_PULL:-0}" = "1" ]; then
  echo "SKIP_PULL=1, using local repository"
else
  if ! git pull; then
    echo "WARN: git pull 失败，继续使用本地仓库。"
  fi
fi

echo "== sync vla_bridge =="
cp -r "$SRC/." "$DST/" || return 1

cd "$WS" || return 1

echo "== source ROS2 =="
source /opt/ros/humble/setup.bash || return 1

echo "== colcon build vla_bridge =="
if ! colcon build --symlink-install --packages-select vla_bridge; then
  echo "ERROR: colcon build 失败。"
  return 1
fi

echo "== source workspace =="
source "$WS/install/setup.bash" || return 1

echo "== ready =="
echo "当前目录: $(pwd)"
echo "可以直接运行 ros2 run / ros2 launch。"
