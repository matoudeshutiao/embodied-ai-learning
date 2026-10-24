import os
import cv2
import random
import numpy as np
import pybullet as p
import pybullet_data
from math import pi, tan, radians, degrees
from ultralytics import YOLO

# ============================================================
# YOLO 第 7 步 · 离线验证
#   1) 框中心 + 深度 → 世界三维点，和真值比较
#   2) 框内深度点云顶面拟合 → yaw，和真值比较
#   3) 用估计 yaw 构造夹爪朝向，检查 IK 可达性
# 不移动机械臂去抓取，只做验证
# ============================================================

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
OUT_DIR = os.path.join(BASE_DIR, "yolo_lesson", "step7_validate")
os.makedirs(OUT_DIR, exist_ok=True)
BEST = os.path.join(BASE_DIR, "yolo_lesson", "runs", "full30", "weights", "best.pt")

random.seed(7)
np.random.seed(7)
model = YOLO(BEST)

p.connect(p.DIRECT)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.8)
p.setTimeStep(1.0 / 240.0)
p.loadURDF("plane.urdf")
panda = p.loadURDF("franka_panda/panda.urdf", useFixedBase=True)

CUBE_SCALE = 1.2
CUBE_HALF = 0.05 * CUBE_SCALE / 2.0
COLOR_RGBA = {"red": [1, 0, 0, 1], "green": [0, 1, 0, 1], "blue": [0, 0, 1, 1]}
NAMES = ["red", "green", "blue"]

cube = p.loadURDF("cube_small.urdf", [0.4, 0, 0.04],
                  globalScaling=CUBE_SCALE)
default_pose = [0, -0.785, 0, -2.356, 0, 1.571, 0.785]
for i in range(7):
    p.resetJointState(panda, i, default_pose[i])
ee_idx = 11
finger_joints = [i for i in range(p.getNumJoints(panda))
                 if "finger" in p.getJointInfo(panda, i)[1].decode()]
for f in finger_joints:
    p.setJointMotorControl2(panda, f, p.POSITION_CONTROL, 0.04, force=200)

WIDTH, HEIGHT = 640, 480
FOV = 60.0
NEAR, FAR = 0.05, 5.0
fx = (HEIGHT / 2.0) / tan(radians(FOV) / 2.0)
fy = fx
cx = WIDTH / 2.0
cy = HEIGHT / 2.0


def set_arm_to_hand(hand_target, orn):
    jp = p.calculateInverseKinematics(
        panda, ee_idx, hand_target, orn,
        maxNumIterations=200, residualThreshold=1e-8)
    for i in range(7):
        p.resetJointState(panda, i, jp[i])
    p.stepSimulation()
    return p.getLinkState(panda, ee_idx)[4]


def camera_from_hand(hand):
    return ([hand[0] - 0.12, hand[1], hand[2] + 0.12],
            [hand[0], hand[1], 0.0],
            [0.0, 1.0, 0.0])


def camera_basis(eye, target, up):
    forward = np.array(target, float) - np.array(eye, float)
    forward /= np.linalg.norm(forward)
    right = np.cross(forward, np.array(up, float))
    right /= np.linalg.norm(right)
    cam_up = np.cross(right, forward)
    return forward, right, cam_up


def backproject(us, vs, depth, eye, forward, right, cam_up):
    Z = depth[vs, us]
    X = (us - cx) * Z / fx
    Y = -(vs - cy) * Z / fy
    return (np.array(eye, float)
            + right[None, :] * X[:, None]
            + cam_up[None, :] * Y[:, None]
            + forward[None, :] * Z[:, None]), Z


results = []
tiles = []

for idx in range(12):
    color = NAMES[idx % 3]
    p.changeVisualShape(cube, -1, rgbaColor=COLOR_RGBA[color])
    x = random.uniform(0.30, 0.45)
    y = random.uniform(-0.12, 0.12)
    yaw_true = random.uniform(-pi, pi)
    orn_cube = p.getQuaternionFromEuler([0, 0, yaw_true])
    p.resetBasePositionAndOrientation(
        cube, [x, y, CUBE_HALF + 0.002], orn_cube)
    for _ in range(15):
        p.stepSimulation()

    look = [x + random.uniform(-0.06, 0.06),
            y + random.uniform(-0.06, 0.06),
            random.uniform(0.35, 0.45)]
    hand = set_arm_to_hand(look, p.getQuaternionFromEuler([0, pi, 0]))
    eye, target, up = camera_from_hand(hand)
    view = p.computeViewMatrix(eye, target, up)
    proj = p.computeProjectionMatrixFOV(FOV, WIDTH / HEIGHT, NEAR, FAR)
    _, _, rgb_pix, depth_pix, _ = p.getCameraImage(
        WIDTH, HEIGHT, viewMatrix=view, projectionMatrix=proj,
        renderer=p.ER_TINY_RENDERER)
    rgb = np.reshape(np.array(rgb_pix, np.uint8),
                     (HEIGHT, WIDTH, 4))[:, :, :3]
    bgr = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
    zbuf = np.reshape(np.array(depth_pix, np.float32), (HEIGHT, WIDTH))
    depth = FAR * NEAR / (FAR - (FAR - NEAR) * zbuf)
    forward, right, cam_up = camera_basis(eye, target, up)

    r = model.predict(bgr, conf=0.25, verbose=False, device=0, imgsz=640)[0]
    if len(r.boxes) == 0:
        print("%02d %-5s DETECTION MISS" % (idx, color))
        results.append(None)
        continue
    k = int(r.boxes.conf.argmax())
    box = r.boxes.xyxy[k].cpu().numpy().tolist()
    cls = int(r.boxes.cls[k])
    conf = float(r.boxes.conf[k])
    x1, y1, x2, y2 = box

    # 1) 框中心像素 + 深度 → 世界点
    u = int(np.clip(round((x1 + x2) / 2), 0, WIDTH - 1))
    v = int(np.clip(round((y1 + y2) / 2), 0, HEIGHT - 1))
    if zbuf[v, u] >= 0.999:
        # 中心点无效时取框内有效深度的中位数
        uu, vv = np.meshgrid(np.arange(int(x1), int(x2)),
                             np.arange(int(y1), int(y2)))
        uu = uu.ravel(); vv = vv.ravel()
        ok = zbuf[vv, uu] < 0.999
        if ok.sum() == 0:
            results.append(None); continue
        u = int(np.median(uu[ok])); v = int(np.median(vv[ok]))
    p3, _ = backproject(np.array([u]), np.array([v]), depth,
                        eye, forward, right, cam_up)
    p3 = p3[0]
    truth = np.array(p.getBasePositionAndOrientation(cube)[0])
    err_xy = float(np.hypot(p3[0] - truth[0], p3[1] - truth[1]))
    err_z = float(p3[2] - (truth[2] + CUBE_HALF))  # 与顶面比

    # 2) 框内点云 → 顶面 → yaw
    yy, xx = np.mgrid[0:HEIGHT, 0:WIDTH]
    in_box = ((xx >= x1) & (xx <= x2) & (yy >= y1) & (yy <= y2) &
              (zbuf < 0.999))
    us = xx[in_box]; vs = yy[in_box]
    pts, Z = backproject(us, vs, depth, eye, forward, right, cam_up)
    # 去掉桌面和远处背景：保留接近框中心深度、且高于桌面的点
    keep = ((np.abs(Z - depth[v, u]) < 0.12) &
            (pts[:, 2] > 0.012) & (pts[:, 2] < 0.15) &
            (np.abs(pts[:, 0] - p3[0]) < 0.12) &
            (np.abs(pts[:, 1] - p3[1]) < 0.12))
    pts = pts[keep]
    yaw_est = None
    if len(pts) >= 20:
        zmax = pts[:, 2].max()
        top = pts[pts[:, 2] > zmax - 0.012]
        if len(top) >= 10:
            rect = cv2.minAreaRect(top[:, :2].astype(np.float32))
            yaw_est = rect[2] % 90.0
    yaw_true_deg = degrees(yaw_true) % 90.0
    yaw_err = None
    if yaw_est is not None:
        d = abs(yaw_est - yaw_true_deg)
        yaw_err = min(d, 90.0 - d)

    # 3) IK：用估计 yaw 构造夹爪朝向，检查可达
    ik_err = None
    if yaw_est is not None:
        grasp_orn = p.getQuaternionFromEuler([0, pi, radians(yaw_est)])
        target_pos = [float(p3[0]), float(p3[1]), 0.30]
        jp = p.calculateInverseKinematics(
            panda, ee_idx, target_pos, grasp_orn,
            maxNumIterations=200, residualThreshold=1e-8)
        for i in range(7):
            p.resetJointState(panda, i, jp[i])
        p.stepSimulation()
        ee = p.getLinkState(panda, ee_idx)[4]
        ik_err = float(np.linalg.norm(np.array(ee) - np.array(target_pos)))

    results.append({
        "idx": idx, "color": color, "conf": conf, "cls": cls,
        "err_xy": err_xy, "err_z": err_z, "yaw_true": yaw_true_deg,
        "yaw_est": yaw_est, "yaw_err": yaw_err, "ik_err": ik_err,
    })
    print("%02d %-5s conf=%.2f xy_err=%.3fm z_err=%+.3fm "
          "yaw_true=%5.1f yaw_est=%s yaw_err=%s ik_err=%s"
          % (idx, color, conf, err_xy, err_z, yaw_true_deg,
             "%.1f" % yaw_est if yaw_est is not None else "None",
             "%.1f" % yaw_err if yaw_err is not None else "None",
             "%.3f" % ik_err if ik_err is not None else "None"))

    vis = bgr.copy()
    cv2.rectangle(vis, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)
    cv2.circle(vis, (u, v), 4, (0, 0, 255), -1)
    cv2.putText(vis, "%s %.2f" % (color, conf), (int(x1), max(16, int(y1) - 5)),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
    cv2.putText(vis, "yawE %s" % ("%.0f" % yaw_est if yaw_est is not None else "-"),
                (8, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
    tiles.append(cv2.resize(vis, (320, 240)))

if tiles:
    rows = (len(tiles) + 3) // 4
    canvas = np.full((rows * 240, 4 * 320, 3), 245, np.uint8)
    for i, t in enumerate(tiles):
        r, c = divmod(i, 4)
        canvas[r * 240:(r + 1) * 240, c * 320:(c + 1) * 320] = t
    cv2.imwrite(os.path.join(OUT_DIR, "validate_montage.png"), canvas)
    print("拼图:", os.path.join(OUT_DIR, "validate_montage.png"))

ok = [x for x in results if x is not None]
if ok:
    print("\n===== 汇总 =====")
    print("有效样本:", len(ok), "/ 12")
    print("平均 xy 误差: %.3f m" % (sum(x["err_xy"] for x in ok) / len(ok)))
    print("平均 z 误差(顶面): %+.3f m" % (sum(x["err_z"] for x in ok) / len(ok)))
    ye = [x["yaw_err"] for x in ok if x["yaw_err"] is not None]
    if ye:
        print("平均 yaw 误差: %.1f 度" % (sum(ye) / len(ye)))
    ik = [x["ik_err"] for x in ok if x["ik_err"] is not None]
    if ik:
        print("IK 可达(<1cm)比例: %d/%d" % (sum(1 for e in ik if e < 0.01), len(ik)))
        print("平均 IK 误差: %.3f m" % (sum(ik) / len(ik)))

p.disconnect()
