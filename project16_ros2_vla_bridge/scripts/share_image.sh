#!/usr/bin/env bash
# 把虚拟机里的图片上传到 GitHub，供 Claude 在主机端 pull 后直接查看。
#
# 用法：
#   bash scripts/share_image.sh <图片路径> [更多图片...]
#   bash scripts/share_image.sh --latest
#
# --latest 会取 project16_ros2_vla_bridge/output/ 里最新的图片。
# 图片会被复制到 experiments/shared_images/（该目录不受 .gitignore 影响）。

set -euo pipefail

# 从脚本自身位置找仓库，不依赖当前工作目录
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO="$(git -C "$SCRIPT_DIR" rev-parse --show-toplevel 2>/dev/null || true)"
if [ -z "$REPO" ]; then
  echo "ERROR: 脚本不在 git 仓库里"
  exit 1
fi

DEST="$REPO/project16_ros2_vla_bridge/experiments/shared_images"
mkdir -p "$DEST"

if [ "$#" -eq 0 ]; then
  echo "用法: bash scripts/share_image.sh <图片路径> [更多图片...]"
  echo "      bash scripts/share_image.sh --latest"
  exit 1
fi

FILES=()
if [ "$1" = "--latest" ]; then
  LATEST="$(find "$REPO/project16_ros2_vla_bridge/output" \
    -type f \( -iname '*.png' -o -iname '*.jpg' \
    -o -iname '*.jpeg' -o -iname '*.webp' \) \
    -printf '%T@ %p\n' 2>/dev/null \
    | sort -nr | head -1 | cut -d' ' -f2-)"
  if [ -z "$LATEST" ]; then
    echo "ERROR: output/ 里没有找到图片"
    exit 1
  fi
  FILES+=("$LATEST")
else
  for path in "$@"; do
    if [ ! -f "$path" ]; then
      echo "ERROR: 文件不存在: $path"
      exit 1
    fi
    FILES+=("$path")
  done
fi

STAMP="$(date +%Y%m%d_%H%M%S)"
ADDED=()
index=1
for path in "${FILES[@]}"; do
  base="$(basename "$path")"
  out="$DEST/${STAMP}_${index}_${base}"
  while [ -e "$out" ]; do
    index=$((index + 1))
    out="$DEST/${STAMP}_${index}_${base}"
  done
  cp "$path" "$out"
  ADDED+=("$out")
  echo "copied: $out"
  index=$((index + 1))
done

REL=()
for path in "${ADDED[@]}"; do
  REL+=("${path#"$REPO"/}")
done

cd "$REPO"
git add "${REL[@]}"
if git diff --cached --quiet -- "${REL[@]}"; then
  echo "没有新内容需要提交"
  exit 0
fi

# 只提交这次复制进来的图片，不带上仓库里其他已暂存的改动
git commit -m "share: upload images ${STAMP}" -- "${REL[@]}"

if ! git push; then
  echo "ERROR: push 失败。先在仓库里 git pull，再重跑这条命令。"
  exit 1
fi

echo
echo "OK. 已推送到 GitHub。仓库相对路径："
for path in "${REL[@]}"; do
  echo "  $path"
done
