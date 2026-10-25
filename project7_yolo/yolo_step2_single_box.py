import os
import cv2
import numpy as np
import pybullet as p
import pybullet_data
from math import pi, tan, radians

# ============================================================
# YOLO 数据集生成 · 第 2 步：单张图 + 单个框
# 目标：把方块的 8 个三维角点投影成二维框，写出一行 YOLO 标注
# ============================================================

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT_DIR = os.path.dirname(BASE_DIR)
OUT_DIR = os.path.join(ROOT_DIR, "yolo_lesson", "step2")
IMG_DIR = os.path.join(OUT_DIR, "images")
LBL_DIR = os.path.join(OUT_DIR, "labels")
ANN_DIR = os.path.join(OUT_DIR, "annotated")
for d in (IMG_DIR, LBL_DIR, ANN_DIR):
    os.makedirs(d, exist_ok=True)

# ---------------- 场景 ----------------
p.connect(p.DIRECT)          # 只生成数据，不需要 GUI
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.8)
p.setTimeStep(1.0 / 240.0)
p.loadURDF("plane.urdf")
panda = p.loadURDF("franka_panda/panda.urdf", useFixedBase=True)

CUBE_SCALE = 1.2
CUBE_HALF = 0.05 * CUBE_SCALE / 2.0   # 0.03m
CLASS_ID = 0
CLASS_NAME = "red"

cube = p.loadURDF("cube_small.urdf",
                  [0.40, 0.0, CUBE_HALF + 0.001],
                  globalScaling=CUBE_SCALE)
p.changeVisualShape(cube, -1, rgbaColor=[1, 0, 0, 1])
p.resetBasePositionAndOrientation(
    cube, [0.40, 0.0, CUBE_HALF + 0.001], [0, 0, 0, 1])
p.stepSimulation()

# ---------------- 机械臂摆到固定观察位 ----------------
default_pose = [0, -0.785, 0, -2.356, 0, 1.571, 0.785]
for i in range(7):
    p.resetJointState(panda, i, default_pose[i])
ee_idx = 11
target_orn = p.getQuaternionFromEuler([0, pi, 0])
jp = p.calculateInverseKinematics(
    panda, ee_idx, [0.40, 0.0, 0.40], target_orn,
    maxNumIterations=200, residualThreshold=1e-8)
for i in range(7):
    p.resetJointState(panda, i, jp[i])
p.stepSimulation()

# ---------------- eye-in-hand 相机 ----------------
WIDTH, HEIGHT = 640, 480
FOV = 60.0
NEAR, FAR = 0.05, 5.0
fx = (HEIGHT / 2.0) / tan(radians(FOV) / 2.0)
fy = fx
cx = WIDTH / 2.0
cy = HEIGHT / 2.0

hand = p.getLinkState(panda, ee_idx)[4]
eye = [hand[0] - 0.12, hand[1], hand[2] + 0.12]
cam_target = [hand[0], hand[1], 0.0]
cam_up_hint = [0.0, 1.0, 0.0]

view = p.computeViewMatrix(eye, cam_target, cam_up_hint)
proj = p.computeProjectionMatrixFOV(FOV, WIDTH / HEIGHT, NEAR, FAR)
_, _, rgb_pix, _, _ = p.getCameraImage(
    WIDTH, HEIGHT, viewMatrix=view, projectionMatrix=proj,
    renderer=p.ER_TINY_RENDERER)
rgb = np.reshape(np.array(rgb_pix, dtype=np.uint8),
                 (HEIGHT, WIDTH, 4))[:, :, :3]
bgr = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)

# ---------------- 投影用的相机基向量 ----------------
forward = np.array(cam_target, dtype=float) - np.array(eye, dtype=float)
forward /= np.linalg.norm(forward)
right = np.cross(forward, np.array(cam_up_hint, dtype=float))
right /= np.linalg.norm(right)
cam_up = np.cross(right, forward)


def project_to_pixel(point):
    """世界点 → 像素坐标 (u, v) 和相机前方深度 Zc"""
    d = np.array(point, dtype=float) - np.array(eye, dtype=float)
    xc = float(np.dot(d, right))
    yc = float(np.dot(d, cam_up))
    zc = float(np.dot(d, forward))
    if zc <= 1e-6:
        return None
    u = cx + fx * xc / zc
    v = cy - fy * yc / zc
    return u, v, zc


# ---------------- 方块 8 个角点 → 像素框 ----------------
pos, orn = p.getBasePositionAndOrientation(cube)
rot = np.array(p.getMatrixFromQuaternion(orn)).reshape(3, 3)
local_corners = np.array([
    [sx * CUBE_HALF, sy * CUBE_HALF, sz * CUBE_HALF]
    for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)
], dtype=float)
world_corners = (np.array(pos)[None, :] + local_corners @ rot.T)

print("相机 eye:", np.round(eye, 3))
print("相机 target:", np.round(cam_target, 3))
print("内参 fx=fy=%.3f cx=%.1f cy=%.1f" % (fx, cx, cy))
print("\n8 个角点的投影结果 (u, v, Zc):")
projected = []
for i, corner in enumerate(world_corners):
    res = project_to_pixel(corner)
    if res is None:
        print("  角点", i, "在相机后方！")
        continue
    u, v, zc = res
    projected.append((u, v))
    print("  角点%d 世界=%s -> u=%7.2f v=%7.2f Zc=%.3f"
          % (i, np.round(corner, 3), u, v, zc))

projected = np.array(projected)
u_min, v_min = projected.min(axis=0)
u_max, v_max = projected.max(axis=0)

# 裁剪到图像范围内
u_min_c = max(0.0, u_min)
v_min_c = max(0.0, v_min)
u_max_c = min(WIDTH - 1.0, u_max)
v_max_c = min(HEIGHT - 1.0, v_max)

box_w = u_max_c - u_min_c
box_h = v_max_c - v_min_c
cx_pix = (u_min_c + u_max_c) / 2.0
cy_pix = (v_min_c + v_max_c) / 2.0

label_cx = cx_pix / WIDTH
label_cy = cy_pix / HEIGHT
label_w = box_w / WIDTH
label_h = box_h / HEIGHT

print("\n像素框: x_min=%.2f y_min=%.2f x_max=%.2f y_max=%.2f"
      % (u_min_c, v_min_c, u_max_c, v_max_c))
print("像素框宽高: %.2f x %.2f" % (box_w, box_h))
print("YOLO 标签: %d %.6f %.6f %.6f %.6f"
      % (CLASS_ID, label_cx, label_cy, label_w, label_h))

# ---------------- 保存图像与标签 ----------------
img_path = os.path.join(IMG_DIR, "000.png")
lbl_path = os.path.join(LBL_DIR, "000.txt")
cv2.imwrite(img_path, bgr)
with open(lbl_path, "w", encoding="utf-8") as f:
    f.write("%d %.6f %.6f %.6f %.6f\n"
            % (CLASS_ID, label_cx, label_cy, label_w, label_h))

# ---------------- 把框画回图片 ----------------
vis = bgr.copy()
p1 = (int(round(u_min_c)), int(round(v_min_c)))
p2 = (int(round(u_max_c)), int(round(v_max_c)))
cv2.rectangle(vis, p1, p2, (0, 255, 0), 2)

# 八个角点也画出来，方便检查投影
for (u, v) in projected:
    cv2.circle(vis, (int(round(u)), int(round(v))), 3, (0, 255, 255), -1)

text1 = "%s (class %d)" % (CLASS_NAME, CLASS_ID)
text2 = "cx=%.3f cy=%.3f w=%.3f h=%.3f" % (label_cx, label_cy, label_w, label_h)
cv2.putText(vis, text1, (10, 28), cv2.FONT_HERSHEY_SIMPLEX,
            0.7, (0, 255, 0), 2)
cv2.putText(vis, text2, (10, 56), cv2.FONT_HERSHEY_SIMPLEX,
            0.55, (0, 255, 0), 2)
ann_path = os.path.join(ANN_DIR, "000.png")
cv2.imwrite(ann_path, vis)

print("\n已保存:")
print("  图像:", img_path)
print("  标签:", lbl_path)
print("  画框图:", ann_path)
print("标签文件内容:", open(lbl_path, encoding="utf-8").read().strip())

p.disconnect()
