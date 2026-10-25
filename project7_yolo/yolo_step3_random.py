import os
import cv2
import random
import numpy as np
import pybullet as p
import pybullet_data
from math import pi, tan, radians

# ============================================================
# YOLO 数据集生成 · 第 3 步：随机位置 / 颜色 / 朝向 / 相机位姿
# 先生成 10 张，审计框是否正确
# ============================================================

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT_DIR = os.path.dirname(BASE_DIR)
OUT_DIR = os.path.join(ROOT_DIR, "yolo_lesson", "step3")
IMG_DIR = os.path.join(OUT_DIR, "images")
LBL_DIR = os.path.join(OUT_DIR, "labels")
ANN_DIR = os.path.join(OUT_DIR, "annotated")
FROM_LBL_DIR = os.path.join(OUT_DIR, "from_labels")
for d in (IMG_DIR, LBL_DIR, ANN_DIR, FROM_LBL_DIR):
    os.makedirs(d, exist_ok=True)

random.seed(7)   # 方便复现；做正式数据集时换种子即可

# ---------------- 场景 ----------------
p.connect(p.DIRECT)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.8)
p.setTimeStep(1.0 / 240.0)
p.loadURDF("plane.urdf")
panda = p.loadURDF("franka_panda/panda.urdf", useFixedBase=True)

CUBE_SCALE = 1.2
CUBE_HALF = 0.05 * CUBE_SCALE / 2.0
COLORS = {
    "red":   (0, [1, 0, 0, 1]),
    "green": (1, [0, 1, 0, 1]),
    "blue":  (2, [0, 0, 1, 1]),
}
COLOR_NAMES = ["red", "green", "blue"]

cube = p.loadURDF("cube_small.urdf", [0.4, 0, 0.04],
                  globalScaling=CUBE_SCALE)

# ---------------- 机械臂 / 相机参数 ----------------
default_pose = [0, -0.785, 0, -2.356, 0, 1.571, 0.785]
for i in range(7):
    p.resetJointState(panda, i, default_pose[i])
ee_idx = 11
finger_joints = [i for i in range(p.getNumJoints(panda))
                 if "finger" in p.getJointInfo(panda, i)[1].decode()]
target_orn = p.getQuaternionFromEuler([0, pi, 0])
for f in finger_joints:
    p.setJointMotorControl2(panda, f, p.POSITION_CONTROL, 0.04, force=200)

WIDTH, HEIGHT = 640, 480
FOV = 60.0
NEAR, FAR = 0.05, 5.0
fx = (HEIGHT / 2.0) / tan(radians(FOV) / 2.0)
fy = fx
cx = WIDTH / 2.0
cy = HEIGHT / 2.0


def set_arm_to_hand(hand_target):
    jp = p.calculateInverseKinematics(
        panda, ee_idx, hand_target, target_orn,
        maxNumIterations=200, residualThreshold=1e-8)
    for i in range(7):
        p.resetJointState(panda, i, jp[i])
    p.stepSimulation()
    return p.getLinkState(panda, ee_idx)[4]


def camera_from_hand(hand):
    eye = [hand[0] - 0.12, hand[1], hand[2] + 0.12]
    target = [hand[0], hand[1], 0.0]
    up = [0.0, 1.0, 0.0]
    return eye, target, up


def project_points(points, eye, target, up):
    forward = np.array(target, dtype=float) - np.array(eye, dtype=float)
    forward /= np.linalg.norm(forward)
    right = np.cross(forward, np.array(up, dtype=float))
    right /= np.linalg.norm(right)
    cam_up = np.cross(right, forward)

    out = []
    for pt in points:
        d = np.array(pt, dtype=float) - np.array(eye, dtype=float)
        Xc = float(np.dot(d, right))
        Yc = float(np.dot(d, cam_up))
        Zc = float(np.dot(d, forward))
        if Zc <= 1e-6:
            return None
        u = cx + fx * Xc / Zc
        v = cy - fy * Yc / Zc
        out.append((u, v, Zc))
    return out


def cube_corners():
    pos, orn = p.getBasePositionAndOrientation(cube)
    rot = np.array(p.getMatrixFromQuaternion(orn)).reshape(3, 3)
    local = np.array([[sx * CUBE_HALF, sy * CUBE_HALF, sz * CUBE_HALF]
                      for sx in (-1, 1) for sy in (-1, 1)
                      for sz in (-1, 1)], dtype=float)
    return np.array(pos)[None, :] + local @ rot.T


def render_and_project(hand):
    eye, target, up = camera_from_hand(hand)
    view = p.computeViewMatrix(eye, target, up)
    proj = p.computeProjectionMatrixFOV(FOV, WIDTH / HEIGHT, NEAR, FAR)
    _, _, rgb_pix, _, _ = p.getCameraImage(
        WIDTH, HEIGHT, viewMatrix=view, projectionMatrix=proj,
        renderer=p.ER_TINY_RENDERER)
    rgb = np.reshape(np.array(rgb_pix, dtype=np.uint8),
                     (HEIGHT, WIDTH, 4))[:, :, :3]

    corners = cube_corners()
    projected = project_points(corners, eye, target, up)
    if projected is None:
        return None, None, None
    us = np.array([q[0] for q in projected])
    vs = np.array([q[1] for q in projected])
    if us.min() < 2 or vs.min() < 2 or us.max() > WIDTH - 3 or vs.max() > HEIGHT - 3:
        return None, None, None   # 框贴边/被裁，重采
    x1, x2 = float(us.min()), float(us.max())
    y1, y2 = float(vs.min()), float(vs.max())
    if (x2 - x1) < 12 or (y2 - y1) < 12:
        return None, None, None   # 太小，丢弃
    return rgb, (x1, y1, x2, y2), np.array(projected)


def build_annotated(bgr, box, label, color_name, idx):
    vis = bgr.copy()
    x1, y1, x2, y2 = box
    cv2.rectangle(vis, (round(x1), round(y1)),
                  (round(x2), round(y2)), (0, 255, 0), 2)
    cls, ncx, ncy, nw, nh = label
    cv2.putText(vis, "%s cls=%d" % (color_name, cls), (8, 24),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
    cv2.putText(vis, "cx=%.3f cy=%.3f w=%.3f h=%.3f"
                % (ncx, ncy, nw, nh), (8, 50),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
    cv2.imwrite(os.path.join(ANN_DIR, "%03d.png" % idx), vis)
    return vis


def build_from_label(bgr, label, idx):
    """只根据 txt 反算像素框，用来验证标签编码"""
    cls, ncx, ncy, nw, nh = label
    x1 = (ncx - nw / 2) * WIDTH
    x2 = (ncx + nw / 2) * WIDTH
    y1 = (ncy - nh / 2) * HEIGHT
    y2 = (ncy + nh / 2) * HEIGHT
    vis = bgr.copy()
    cv2.rectangle(vis, (round(x1), round(y1)),
                  (round(x2), round(y2)), (255, 0, 0), 1)
    cv2.imwrite(os.path.join(FROM_LBL_DIR, "%03d.png" % idx), vis)
    return vis


# ---------------- 生成 10 张 ----------------
records = []
for idx in range(10):
    color_name = COLOR_NAMES[idx % 3]
    cls, rgba = COLORS[color_name]
    p.changeVisualShape(cube, -1, rgbaColor=rgba)

    for attempt in range(60):
        cx3 = random.uniform(0.30, 0.45)
        cy3 = random.uniform(-0.12, 0.12)
        yaw = random.uniform(-pi, pi)
        p.resetBasePositionAndOrientation(
            cube, [cx3, cy3, CUBE_HALF + 0.002],
            p.getQuaternionFromEuler([0, 0, yaw]))
        for _ in range(20):
            p.stepSimulation()

        hand_target = [cx3 + random.uniform(-0.06, 0.06),
                       cy3 + random.uniform(-0.06, 0.06),
                       random.uniform(0.35, 0.45)]
        hand = set_arm_to_hand(hand_target)
        rgb, box, projected = render_and_project(hand)
        if rgb is not None:
            break
    else:
        print("第 %d 张重采失败，跳过" % idx)
        continue

    x1, y1, x2, y2 = box
    ncx = (x1 + x2) / 2 / WIDTH
    ncy = (y1 + y2) / 2 / HEIGHT
    nw = (x2 - x1) / WIDTH
    nh = (y2 - y1) / HEIGHT
    label = (cls, ncx, ncy, nw, nh)

    bgr = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
    cv2.imwrite(os.path.join(IMG_DIR, "%03d.png" % idx), bgr)
    with open(os.path.join(LBL_DIR, "%03d.txt" % idx), "w") as f:
        f.write("%d %.6f %.6f %.6f %.6f\n" % label)

    build_annotated(bgr, box, label, color_name, idx)
    build_from_label(bgr, label, idx)
    records.append((idx, color_name, label, box, attempt + 1))
    print("%03d %-5s box=(%.1f,%.1f)-(%.1f,%.1f)  label=(%.4f %.4f %.4f %.4f) 尝试=%d"
          % (idx, color_name, x1, y1, x2, y2, ncx, ncy, nw, nh, attempt + 1))

# ---------------- 审计拼图：投影框 + txt 反算框 ----------------
def make_montage(folder, out_path, title_prefix):
    tiles = []
    for idx in range(10):
        path = os.path.join(folder, "%03d.png" % idx)
        if not os.path.exists(path):
            continue
        im = cv2.imread(path)
        im = cv2.resize(im, (320, 240))
        cv2.putText(im, "%s %03d" % (title_prefix, idx), (8, 232),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 255), 2)
        tiles.append(im)
    if not tiles:
        return
    rows = (len(tiles) + 4) // 5
    canvas = np.full((rows * 240, 5 * 320, 3), 245, np.uint8)
    for i, tile in enumerate(tiles):
        r, c = divmod(i, 5)
        canvas[r * 240:(r + 1) * 240, c * 320:(c + 1) * 320] = tile
    cv2.imwrite(out_path, canvas)
    print("审计拼图:", out_path)


make_montage(ANN_DIR, os.path.join(OUT_DIR, "audit_projected.png"), "projected")
make_montage(FROM_LBL_DIR, os.path.join(OUT_DIR, "audit_from_label.png"), "from_label")

valid = [r for r in records]
print("\n成功生成 %d/10 张" % len(valid))
from collections import Counter
print("类别分布:", Counter([r[1] for r in valid]))
p.disconnect()
