import os
import cv2
import random
import numpy as np
import pybullet as p
import pybullet_data
from collections import Counter
from math import pi, tan, radians

# ============================================================
# YOLO 数据集 · 第 4 步：200 张随机图像 + 标签 + 数据集审计
# 类别：0=red, 1=green, 2=blue
# 标签：class cx cy w h（归一化，AABB）
# ============================================================

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.join(BASE_DIR, "yolo_lesson", "dataset")
TRAIN_IMG = os.path.join(ROOT, "images", "train")
VAL_IMG = os.path.join(ROOT, "images", "val")
TRAIN_LBL = os.path.join(ROOT, "labels", "train")
VAL_LBL = os.path.join(ROOT, "labels", "val")
AUDIT_DIR = os.path.join(ROOT, "audit")
for d in (TRAIN_IMG, VAL_IMG, TRAIN_LBL, VAL_LBL, AUDIT_DIR):
    os.makedirs(d, exist_ok=True)

SEED = 2026
random.seed(SEED)
np.random.seed(SEED)

# ---------------- 场景 ----------------
p.connect(p.DIRECT)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.8)
p.setTimeStep(1.0 / 240.0)
plane = p.loadURDF("plane.urdf")
panda = p.loadURDF("franka_panda/panda.urdf", useFixedBase=True)

CUBE_SCALE = 1.2
CUBE_HALF = 0.05 * CUBE_SCALE / 2.0
COLOR_NAMES = ["red", "green", "blue"]
COLOR_IDS = {"red": 0, "green": 1, "blue": 2}
COLOR_RGBA = {
    "red": [1, 0, 0, 1],
    "green": [0, 1, 0, 1],
    "blue": [0, 0, 1, 1],
}
HSV_RANGES = {
    "red": [((0, 100, 40), (12, 255, 255)),
            ((168, 100, 40), (180, 255, 255))],
    "green": [((38, 100, 40), (88, 255, 255))],
    "blue": [((98, 100, 40), (138, 255, 255))],
}

cube = p.loadURDF("cube_small.urdf", [0.4, 0, 0.04],
                  globalScaling=CUBE_SCALE)

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
        return None, None
    us = np.array([q[0] for q in projected])
    vs = np.array([q[1] for q in projected])
    if us.min() < 3 or vs.min() < 3 or us.max() > WIDTH - 4 or vs.max() > HEIGHT - 4:
        return None, None
    x1, x2 = float(us.min()), float(us.max())
    y1, y2 = float(vs.min()), float(vs.max())
    if (x2 - x1) < 12 or (y2 - y1) < 12:
        return None, None
    return rgb, (x1, y1, x2, y2)


def photometric_augment(rgb):
    """只扰动亮度/对比度/gamma/噪声，不改色相，所以类别标签不变"""
    img = rgb.astype(np.float32)
    brightness = random.uniform(-18, 18)
    contrast = random.uniform(0.85, 1.15)
    gamma = random.uniform(0.85, 1.15)
    img = (img - 128.0) * contrast + 128.0 + brightness
    img = np.clip(img, 0, 255) / 255.0
    img = 255.0 * np.power(img, gamma)
    img += np.random.normal(0, random.uniform(0, 2.5), img.shape)
    return np.clip(img, 0, 255).astype(np.uint8)


def random_background():
    base = random.uniform(0.45, 0.72)
    rgba = [base * random.uniform(0.97, 1.03),
            base * random.uniform(0.97, 1.03),
            base * random.uniform(0.97, 1.03), 1.0]
    p.changeVisualShape(plane, -1, rgbaColor=rgba)


# ---------------- 200 张的配置与划分 ----------------
N = 200
colors = (COLOR_NAMES * 67)[:N]          # 67/67/66
random.shuffle(colors)
splits = ["train"] * 160 + ["val"] * 40
random.shuffle(splits)

records = []
for idx in range(N):
    color = colors[idx]
    split = splits[idx]
    cls = COLOR_IDS[color]
    p.changeVisualShape(cube, -1, rgbaColor=COLOR_RGBA[color])
    random_background()

    for attempt in range(80):
        x = random.uniform(0.30, 0.45)
        y = random.uniform(-0.12, 0.12)
        yaw = random.uniform(-pi, pi)
        p.resetBasePositionAndOrientation(
            cube, [x, y, CUBE_HALF + 0.002],
            p.getQuaternionFromEuler([0, 0, yaw]))
        for _ in range(15):
            p.stepSimulation()

        hand_target = [x + random.uniform(-0.06, 0.06),
                       y + random.uniform(-0.06, 0.06),
                       random.uniform(0.35, 0.45)]
        hand = set_arm_to_hand(hand_target)
        rgb, box = render_and_project(hand)
        if rgb is not None:
            break
    else:
        print("跳过索引", idx)
        continue

    rgb = photometric_augment(rgb)
    x1, y1, x2, y2 = box
    ncx = (x1 + x2) / 2.0 / WIDTH
    ncy = (y1 + y2) / 2.0 / HEIGHT
    nw = (x2 - x1) / WIDTH
    nh = (y2 - y1) / HEIGHT

    name = "img_%04d" % idx
    img_path = os.path.join(TRAIN_IMG if split == "train" else VAL_IMG,
                            name + ".png")
    lbl_path = os.path.join(TRAIN_LBL if split == "train" else VAL_LBL,
                            name + ".txt")
    cv2.imwrite(img_path, cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR))
    with open(lbl_path, "w") as f:
        f.write("%d %.6f %.6f %.6f %.6f\n" % (cls, ncx, ncy, nw, nh))

    records.append({
        "name": name, "split": split, "color": color, "cls": cls,
        "box": (x1, y1, x2, y2), "label": (ncx, ncy, nw, nh),
        "attempt": attempt + 1, "img": img_path, "lbl": lbl_path,
    })
    if (idx + 1) % 20 == 0:
        print("已生成 %d/%d" % (idx + 1, N))

# ---------------- data.yaml ----------------
yaml_path = os.path.join(ROOT, "data.yaml")
root_posix = ROOT.replace("\\", "/")
with open(yaml_path, "w", encoding="utf-8") as f:
    f.write("path: %s\n" % root_posix)
    f.write("train: images/train\n")
    f.write("val: images/val\n")
    f.write("names:\n")
    f.write("  0: red_block\n")
    f.write("  1: green_block\n")
    f.write("  2: blue_block\n")

# ---------------- 审计 ----------------
def color_mask(bgr, color):
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    mask = np.zeros(hsv.shape[:2], dtype=np.uint8)
    for lo, hi in HSV_RANGES[color]:
        mask |= cv2.inRange(hsv, lo, hi)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
    return mask


invalid_labels = 0
coverage_failures = []
for rec in records:
    x1, y1, x2, y2 = rec["box"]
    ncx, ncy, nw, nh = rec["label"]
    if not (0 <= ncx <= 1 and 0 <= ncy <= 1 and 0 < nw <= 1 and 0 < nh <= 1):
        invalid_labels += 1
    img = cv2.imread(rec["img"])
    mask = color_mask(img, rec["color"])
    box_mask = np.zeros(mask.shape, np.uint8)
    box_mask[max(0, int(y1)):min(HEIGHT, int(y2)),
             max(0, int(x1)):min(WIDTH, int(x2))] = 255
    total = int((mask > 0).sum())
    inside = int(((mask > 0) & (box_mask > 0)).sum())
    coverage = inside / total if total else 0.0
    if coverage < 0.95:
        coverage_failures.append((rec["name"], coverage, total))

counts_all = Counter([r["color"] for r in records])
counts_split = {
    "train": Counter([r["color"] for r in records if r["split"] == "train"]),
    "val": Counter([r["color"] for r in records if r["split"] == "val"]),
}
box_ws = [r["box"][2] - r["box"][0] for r in records]
box_hs = [r["box"][3] - r["box"][1] for r in records]

stats_lines = [
    "图像总数: %d" % len(records),
    "train/val: %d / %d" % (
        sum(1 for r in records if r["split"] == "train"),
        sum(1 for r in records if r["split"] == "val")),
    "类别总数: %s" % dict(counts_all),
    "train 类别: %s" % dict(counts_split["train"]),
    "val 类别: %s" % dict(counts_split["val"]),
    "非法标签数: %d" % invalid_labels,
    "框宽 (px): min=%.1f mean=%.1f max=%.1f" %
    (min(box_ws), sum(box_ws) / len(box_ws), max(box_ws)),
    "框高 (px): min=%.1f mean=%.1f max=%.1f" %
    (min(box_hs), sum(box_hs) / len(box_hs), max(box_hs)),
    "颜色掩码覆盖率 <95%% 的图片数: %d" % len(coverage_failures),
]
if coverage_failures:
    stats_lines.append("覆盖率异常样例: %s" % coverage_failures[:5])

print("\n".join(stats_lines))
with open(os.path.join(AUDIT_DIR, "stats.txt"), "w", encoding="utf-8") as f:
    f.write("\n".join(stats_lines) + "\n")

# 随机抽 20 张，把只有 txt 反算出的框画回去
sample = random.sample(records, min(20, len(records)))
tiles = []
for rec in sample:
    img = cv2.imread(rec["img"])
    ncx, ncy, nw, nh = rec["label"]
    x1 = int((ncx - nw / 2) * WIDTH)
    x2 = int((ncx + nw / 2) * WIDTH)
    y1 = int((ncy - nh / 2) * HEIGHT)
    y2 = int((ncy + nh / 2) * HEIGHT)
    cv2.rectangle(img, (x1, y1), (x2, y2), (0, 255, 0), 2)
    cv2.putText(img, "%s %s" % (rec["name"], rec["color"]),
                (8, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 0), 2)
    img = cv2.resize(img, (320, 240))
    tiles.append(img)

canvas = np.full((4 * 240, 5 * 320, 3), 245, np.uint8)
for i, tile in enumerate(tiles):
    r, c = divmod(i, 5)
    canvas[r * 240:(r + 1) * 240, c * 320:(c + 1) * 320] = tile
montage_path = os.path.join(AUDIT_DIR, "audit_montage.png")
cv2.imwrite(montage_path, canvas)

print("\ndata.yaml:", yaml_path)
print("审计拼图:", montage_path)
print("审计统计:", os.path.join(AUDIT_DIR, "stats.txt"))
p.disconnect()
