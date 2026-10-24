import os
import cv2
import random
import numpy as np
import pybullet as p
import pybullet_data
from math import pi, tan, radians
from ultralytics import YOLO

# ============================================================
# YOLO 第 6 步：未见场景泛化测试
# 测试集条件比训练集更极端：
#   - 位置 / 相机高度 / 偏移超出训练范围
#   - 更暗/更亮、更强对比度、更强背景颜色扰动
# ============================================================

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
TEST_ROOT = os.path.join(BASE_DIR, "yolo_lesson", "test_unseen")
IMG_DIR = os.path.join(TEST_ROOT, "images")
LBL_DIR = os.path.join(TEST_ROOT, "labels")
AUDIT_DIR = os.path.join(TEST_ROOT, "audit")
for d in (IMG_DIR, LBL_DIR, AUDIT_DIR):
    os.makedirs(d, exist_ok=True)

BEST = os.path.join(BASE_DIR, "yolo_lesson", "runs", "full30", "weights", "best.pt")
SEED = 606
random.seed(SEED)
np.random.seed(SEED)

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
COLOR_RGBA = {"red": [1, 0, 0, 1],
              "green": [0, 1, 0, 1],
              "blue": [0, 0, 1, 1]}

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
    return ([hand[0] - 0.12, hand[1], hand[2] + 0.12],
            [hand[0], hand[1], 0.0],
            [0.0, 1.0, 0.0])


def project_points(points, eye, target, up):
    forward = np.array(target, float) - np.array(eye, float)
    forward /= np.linalg.norm(forward)
    right = np.cross(forward, np.array(up, float))
    right /= np.linalg.norm(right)
    cam_up = np.cross(right, forward)
    out = []
    for pt in points:
        d = np.array(pt, float) - np.array(eye, float)
        Xc, Yc, Zc = np.dot(d, right), np.dot(d, cam_up), np.dot(d, forward)
        if Zc <= 1e-6:
            return None
        out.append((cx + fx * Xc / Zc, cy - fy * Yc / Zc))
    return out


def cube_corners():
    pos, orn = p.getBasePositionAndOrientation(cube)
    rot = np.array(p.getMatrixFromQuaternion(orn)).reshape(3, 3)
    local = np.array([[sx * CUBE_HALF, sy * CUBE_HALF, sz * CUBE_HALF]
                      for sx in (-1, 1) for sy in (-1, 1)
                      for sz in (-1, 1)], float)
    return np.array(pos)[None, :] + local @ rot.T


def extreme_photometric(rgb):
    """比训练集更极端的亮度/对比度/gamma/噪声；色相基本不变"""
    img = rgb.astype(np.float32)
    img = (img - 128.0) * random.uniform(0.70, 1.30) + 128.0
    img += random.uniform(-35, 35)
    img = np.clip(img, 0, 255) / 255.0
    img = 255.0 * np.power(img, random.uniform(0.70, 1.30))
    img += np.random.normal(0, random.uniform(0, 5.0), img.shape)
    return np.clip(img, 0, 255).astype(np.uint8)


def extreme_background():
    base = random.uniform(0.22, 0.82)
    rgba = [base * random.uniform(0.85, 1.15),
            base * random.uniform(0.85, 1.15),
            base * random.uniform(0.85, 1.15), 1.0]
    p.changeVisualShape(plane, -1, rgbaColor=rgba)


# ---------------- 生成 60 张未见测试图 ----------------
colors = (COLOR_NAMES * 20)[:60]
random.shuffle(colors)
records = []

for idx in range(60):
    color = colors[idx]
    cls = COLOR_IDS[color]
    p.changeVisualShape(cube, -1, rgbaColor=COLOR_RGBA[color])
    extreme_background()

    for attempt in range(100):
        # 超出训练范围：训练是 x 0.30~0.45, y ±0.12, 相机 offset ±0.06, z 0.35~0.45
        x = random.uniform(0.27, 0.48)
        y = random.uniform(-0.16, 0.16)
        yaw = random.uniform(-pi, pi)
        p.resetBasePositionAndOrientation(
            cube, [x, y, CUBE_HALF + 0.002],
            p.getQuaternionFromEuler([0, 0, yaw]))
        for _ in range(15):
            p.stepSimulation()

        hand_target = [x + random.uniform(-0.08, 0.08),
                       y + random.uniform(-0.08, 0.08),
                       random.uniform(0.32, 0.47)]
        hand = set_arm_to_hand(hand_target)
        eye, target, up = camera_from_hand(hand)
        view = p.computeViewMatrix(eye, target, up)
        proj = p.computeProjectionMatrixFOV(FOV, WIDTH / HEIGHT, NEAR, FAR)
        _, _, rgb_pix, _, _ = p.getCameraImage(
            WIDTH, HEIGHT, viewMatrix=view, projectionMatrix=proj,
            renderer=p.ER_TINY_RENDERER)
        rgb = np.reshape(np.array(rgb_pix, np.uint8),
                         (HEIGHT, WIDTH, 4))[:, :, :3]
        projected = project_points(cube_corners(), eye, target, up)
        if projected is None:
            continue
        us = np.array([q[0] for q in projected])
        vs = np.array([q[1] for q in projected])
        if us.min() < 3 or vs.min() < 3 or us.max() > WIDTH - 4 or vs.max() > HEIGHT - 4:
            continue
        x1, x2 = float(us.min()), float(us.max())
        y1, y2 = float(vs.min()), float(vs.max())
        if (x2 - x1) < 12 or (y2 - y1) < 12:
            continue
        break
    else:
        print("跳过", idx)
        continue

    rgb = extreme_photometric(rgb)
    bgr = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
    name = "test_%03d" % idx
    cv2.imwrite(os.path.join(IMG_DIR, name + ".png"), bgr)
    ncx = (x1 + x2) / 2 / WIDTH
    ncy = (y1 + y2) / 2 / HEIGHT
    nw = (x2 - x1) / WIDTH
    nh = (y2 - y1) / HEIGHT
    with open(os.path.join(LBL_DIR, name + ".txt"), "w") as f:
        f.write("%d %.6f %.6f %.6f %.6f\n" % (cls, ncx, ncy, nw, nh))
    records.append((name, color, cls, (x1, y1, x2, y2), (ncx, ncy, nw, nh)))

yaml_path = os.path.join(TEST_ROOT, "data.yaml")
with open(yaml_path, "w", encoding="utf-8") as f:
    f.write("path: %s\n" % TEST_ROOT.replace("\\", "/"))
    f.write("train: images\nval: images\n")
    f.write("names:\n  0: red_block\n  1: green_block\n  2: blue_block\n")

print("测试集生成:", len(records), "张")

# ---------------- 逐张预测 + 指标 ----------------
model = YOLO(BEST)


def iou_xyxy(a, b):
    xa1, ya1, xa2, ya2 = a
    xb1, yb1, xb2, yb2 = b
    ix1, iy1 = max(xa1, xb1), max(ya1, yb1)
    ix2, iy2 = min(xa2, xb2), min(ya2, yb2)
    iw, ih = max(0, ix2 - ix1), max(0, iy2 - iy1)
    inter = iw * ih
    ua = (xa2 - xa1) * (ya2 - ya1) + (xb2 - xb1) * (yb2 - yb1) - inter
    return inter / ua if ua > 0 else 0.0


detected = 0
class_ok = 0
success = 0
ious = []
failures = []
tiles = []

for name, color, cls, gt_box, label in records:
    img_path = os.path.join(IMG_DIR, name + ".png")
    r = model.predict(img_path, conf=0.25, verbose=False, device=0, imgsz=640)[0]
    img = cv2.imread(img_path)
    pred_box = None
    pred_cls = None
    pred_conf = 0.0
    if len(r.boxes) > 0:
        k = int(r.boxes.conf.argmax())
        pred_box = r.boxes.xyxy[k].cpu().numpy().tolist()
        pred_cls = int(r.boxes.cls[k])
        pred_conf = float(r.boxes.conf[k])
        detected += 1
        if pred_cls == cls:
            class_ok += 1
        iou = iou_xyxy(gt_box, pred_box)
        ious.append(iou)
        if pred_cls == cls and iou >= 0.5:
            success += 1
        else:
            failures.append((name, color, "cls" if pred_cls != cls else "iou",
                             round(pred_conf, 3), round(iou, 3)))
    else:
        failures.append((name, color, "miss", 0.0, 0.0))

    # 画 GT（绿）和预测（红）
    vis = img.copy()
    cv2.rectangle(vis, (int(gt_box[0]), int(gt_box[1])),
                  (int(gt_box[2]), int(gt_box[3])), (0, 255, 0), 2)
    if pred_box is not None:
        cv2.rectangle(vis, (int(pred_box[0]), int(pred_box[1])),
                      (int(pred_box[2]), int(pred_box[3])), (0, 0, 255), 2)
        cv2.putText(vis, "%d %.2f" % (pred_cls, pred_conf),
                    (int(pred_box[0]), max(14, int(pred_box[1]) - 4)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
    cv2.putText(vis, name, (6, 20),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1)
    tiles.append(cv2.resize(vis, (320, 240)))

# 12 张审计拼图
sel = random.sample(tiles, min(12, len(tiles)))
canvas = np.full((3 * 240, 4 * 320, 3), 245, np.uint8)
for i, tile in enumerate(sel):
    r, c = divmod(i, 4)
    canvas[r * 240:(r + 1) * 240, c * 320:(c + 1) * 320] = tile
montage_path = os.path.join(AUDIT_DIR, "unseen_predictions.png")
cv2.imwrite(montage_path, canvas)

total = len(records)
print("\n===== 未见测试集结果 =====")
print("总数:", total)
print("有检测的图片:", detected, "/", total)
print("类别正确:", class_ok, "/", detected if detected else 0)
print("类别正确且 IoU>=0.5:", success, "/", total)
if ious:
    print("平均 IoU: %.3f" % (sum(ious) / len(ious)))
print("失败样例:", failures[:8])
print("预测拼图:", montage_path)

# 再用 val 跑一次 mAP
metrics = model.val(data=yaml_path, split="val", conf=0.001, iou=0.5,
                    plots=True,
                    project=os.path.abspath(os.path.join(BASE_DIR, "yolo_lesson", "runs")),
                    name="test_unseen_val", exist_ok=True)
print("\n测试集 mAP50: %.4f | mAP50-95: %.4f"
      % (float(metrics.box.map50), float(metrics.box.map)))

p.disconnect()
