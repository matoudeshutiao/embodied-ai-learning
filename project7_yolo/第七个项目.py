import os
import cv2
import random
import numpy as np
import pybullet as p
import pybullet_data
import time
from math import pi, tan, radians, degrees
from ultralytics import YOLO

# ============================================================
# 第七个项目：YOLO 视觉抓取闭环
#   YOLO 检测 → 框内深度点云 → 顶面中心 + yaw
#   → 移到上方 → 转腕对准 → 下降 → 触觉 → 抬起
# HSV 完全退役，只用 YOLO 检测 + 深度几何
# ============================================================

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT_DIR = os.path.dirname(BASE_DIR)
BEST = os.path.join(ROOT_DIR, "yolo_lesson", "runs", "full30", "weights", "best.pt")
OUT_DIR = os.path.join(ROOT_DIR, "yolo_lesson", "project7")
os.makedirs(OUT_DIR, exist_ok=True)

random.seed(11)
np.random.seed(11)

# ---------------- 场景 ----------------
p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.8)
p.setRealTimeSimulation(0)
p.setTimeStep(1.0 / 240.0)
p.resetDebugVisualizerCamera(cameraDistance=1.3,
                             cameraYaw=45,
                             cameraPitch=-25,
                             cameraTargetPosition=[0.4, 0.0, 0.15])
p.loadURDF("plane.urdf")
panda = p.loadURDF("franka_panda/panda.urdf", useFixedBase=True)

CUBE_SCALE = 1.2
CUBE_HALF = 0.05 * CUBE_SCALE / 2.0
COLOR_RGBA = {"red": [1, 0, 0, 1],
              "green": [0, 1, 0, 1],
              "blue": [0, 0, 1, 1]}
COLOR_NAMES = ["red", "green", "blue"]

# 随机放一个方块（颜色、位置、yaw 都随机）
cube_color = random.choice(COLOR_NAMES)
cube_x = random.uniform(0.30, 0.45)
cube_y = random.uniform(-0.12, 0.12)
cube_yaw = random.uniform(-pi, pi)
cube = p.loadURDF("cube_small.urdf",
                  [cube_x, cube_y, CUBE_HALF + 0.01],
                  globalScaling=CUBE_SCALE)
p.changeVisualShape(cube, -1, rgbaColor=COLOR_RGBA[cube_color])
p.resetBasePositionAndOrientation(
    cube, [cube_x, cube_y, CUBE_HALF + 0.002],
    p.getQuaternionFromEuler([0, 0, cube_yaw]))
for _ in range(300):
    p.stepSimulation()
cube_truth = np.array(p.getBasePositionAndOrientation(cube)[0])
print("方块真值: color=%s pos=(%.3f, %.3f, %.3f) yaw=%.1f deg"
      % (cube_color, cube_truth[0], cube_truth[1], cube_truth[2],
         degrees(cube_yaw) % 90))

# ---------------- 机械臂与相机 ----------------
default_pose = [0, -0.785, 0, -2.356, 0, 1.571, 0.785]
for i in range(7):
    p.resetJointState(panda, i, default_pose[i])
ee_idx = 11
finger_joints = [i for i in range(p.getNumJoints(panda))
                 if "finger" in p.getJointInfo(panda, i)[1].decode()]
base_orn = p.getQuaternionFromEuler([0, pi, 0])

WIDTH, HEIGHT = 640, 480
FOV = 60.0
NEAR, FAR = 0.05, 5.0
fx = (HEIGHT / 2.0) / tan(radians(FOV) / 2.0)
fy = fx
cx = WIDTH / 2.0
cy = HEIGHT / 2.0

model = YOLO(BEST)


def set_gripper(closed):
    for f in finger_joints:
        p.setJointMotorControl2(panda, f, p.POSITION_CONTROL,
                                0.0 if closed else 0.04, force=200)


def move_to(target_pos, orn, steps=200):
    for _ in range(steps):
        joint_pose = p.calculateInverseKinematics(
            panda, ee_idx, target_pos, orn,
            maxNumIterations=200, residualThreshold=1e-8)
        for i in range(7):
            p.setJointMotorControl2(panda, i, p.POSITION_CONTROL,
                                    joint_pose[i], force=200)
        p.stepSimulation()
        time.sleep(1.0 / 240.0)


def camera_from_hand(hand):
    eye = [hand[0] - 0.12, hand[1], hand[2] + 0.12]
    target = [hand[0], hand[1], 0.0]
    up = [0.0, 1.0, 0.0]
    return eye, target, up


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
    pts = (np.array(eye, float)
           + right[None, :] * X[:, None]
           + cam_up[None, :] * Y[:, None]
           + forward[None, :] * Z[:, None])
    return pts, Z


def capture_rgbd():
    hand = p.getLinkState(panda, ee_idx)[4]
    eye, target, up = camera_from_hand(hand)
    view = p.computeViewMatrix(eye, target, up)
    proj = p.computeProjectionMatrixFOV(FOV, WIDTH / HEIGHT, NEAR, FAR)
    try:
        _, _, rgb_pix, depth_pix, _ = p.getCameraImage(
            WIDTH, HEIGHT, viewMatrix=view, projectionMatrix=proj,
            renderer=p.ER_BULLET_HARDWARE_OPENGL)
    except p.error:
        _, _, rgb_pix, depth_pix, _ = p.getCameraImage(
            WIDTH, HEIGHT, viewMatrix=view, projectionMatrix=proj,
            renderer=p.ER_TINY_RENDERER)
    rgb = np.reshape(np.array(rgb_pix, np.uint8),
                     (HEIGHT, WIDTH, 4))[:, :, :3]
    zbuf = np.reshape(np.array(depth_pix, np.float32), (HEIGHT, WIDTH))
    depth = FAR * NEAR / (FAR - (FAR - NEAR) * zbuf)
    return rgb, depth, zbuf, eye, target, up


def detect_cube(tag="detect"):
    """YOLO 检测 + 框内深度点云 → 顶面中心 / yaw / 抓取高度"""
    rgb, depth, zbuf, eye, target, up = capture_rgbd()
    bgr = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
    r = model.predict(bgr, conf=0.25, verbose=False, device=0, imgsz=640)[0]
    if len(r.boxes) == 0:
        cv2.imshow("YOLO grasp - RGB (%s)" % tag, bgr)
        cv2.waitKey(1)
        return None

    k = int(r.boxes.conf.argmax())
    box = r.boxes.xyxy[k].cpu().numpy().tolist()
    cls = int(r.boxes.cls[k])
    conf = float(r.boxes.conf[k])
    x1, y1, x2, y2 = box
    forward, right, cam_up = camera_basis(eye, target, up)

    # 框中心射线的三维点（作为兜底）
    u = int(np.clip(round((x1 + x2) / 2), 0, WIDTH - 1))
    v = int(np.clip(round((y1 + y2) / 2), 0, HEIGHT - 1))
    if zbuf[v, u] >= 0.999:
        return None
    p3, _ = backproject(np.array([u]), np.array([v]), depth,
                        eye, forward, right, cam_up)
    p3 = p3[0]

    # 框内点云
    yy, xx = np.mgrid[0:HEIGHT, 0:WIDTH]
    in_box = ((xx >= x1) & (xx <= x2) & (yy >= y1) & (yy <= y2) &
              (zbuf < 0.999))
    us = xx[in_box]; vs = yy[in_box]
    pts, Z = backproject(us, vs, depth, eye, forward, right, cam_up)
    keep = ((np.abs(Z - depth[v, u]) < 0.12) &
            (pts[:, 2] > 0.012) & (pts[:, 2] < 0.15) &
            (np.abs(pts[:, 0] - p3[0]) < 0.12) &
            (np.abs(pts[:, 1] - p3[1]) < 0.12))
    pts = pts[keep]

    top_center = p3
    z_top = p3[2]
    yaw_est = None
    if len(pts) >= 20:
        zmax = pts[:, 2].max()
        top = pts[pts[:, 2] > zmax - 0.012]
        if len(top) >= 10:
            top_center = np.array([(top[:, 0].min() + top[:, 0].max()) / 2,
                                   (top[:, 1].min() + top[:, 1].max()) / 2,
                                   zmax])
            z_top = zmax
            rect = cv2.minAreaRect(top[:, :2].astype(np.float32))
            yaw_est = radians(rect[2] % 90.0)

    if yaw_est is None:
        print("YOLO 检测到了，但深度顶面点云不足，无法估 yaw")
        return None

    grasp_z = float(np.clip(z_top - CUBE_HALF + 0.0075, 0.03, 0.05))
    info = {
        "color": COLOR_NAMES[cls] if cls < 3 else str(cls),
        "cls": cls, "conf": conf, "box": box,
        "top": top_center, "yaw": yaw_est,
        "grasp_z": grasp_z,
    }

    vis = bgr.copy()
    cv2.rectangle(vis, (int(x1), int(y1)), (int(x2), int(y2)),
                  (0, 255, 0), 2)
    cv2.circle(vis, (u, v), 4, (0, 0, 255), -1)
    cv2.putText(vis, "%s %.2f" % (info["color"], conf),
                (int(x1), max(16, int(y1) - 5)),
                cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 0), 2)
    cv2.putText(vis, "yaw %.1f" % degrees(yaw_est),
                (8, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 255), 2)
    cv2.imshow("YOLO grasp - RGB (%s)" % tag, vis)
    cv2.waitKey(1)
    return info


def tactile_read(block_id):
    contacts = p.getContactPoints(bodyA=panda, bodyB=block_id)
    per = {f: 0.0 for f in finger_joints}
    for c in contacts:
        if c[3] in per:
            per[c[3]] += c[9]
    return per, sum(per.values()), len(contacts)


def is_grasped(block_id):
    per, total, n = tactile_read(block_id)
    return all(per[f] > 5.0 for f in finger_joints), per, total, n


# ---------------- 1. 寻找方块 ----------------
set_gripper(False)
LOOK_POSES = [
    [0.40, 0.00, 0.40],
    [0.35, 0.08, 0.41],
    [0.45, -0.08, 0.41],
    [0.35, -0.08, 0.41],
    [0.45, 0.08, 0.41],
]

info = None
print("开始寻找方块...")
for i, lp in enumerate(LOOK_POSES):
    move_to(lp, base_orn, steps=160)
    info = detect_cube("look %d" % i)
    if info is not None:
        break
if info is None:
    print("所有观察位都没检测到方块，退出")
    cv2.destroyAllWindows()
    p.disconnect()
    raise SystemExit

top = info["top"]
yaw_est = info["yaw"]
print("YOLO: color=%s conf=%.2f | 顶面中心=(%.3f, %.3f, %.3f) | yaw=%.1f deg | grasp_z=%.3f"
      % (info["color"], info["conf"], top[0], top[1], top[2],
         degrees(yaw_est), info["grasp_z"]))
print("位置误差: xy=%.3f m | z=%+.3f m（相对真值顶面）"
      % (np.hypot(top[0] - cube_truth[0], top[1] - cube_truth[1]),
         top[2] - (cube_truth[2] + CUBE_HALF)))

# ---------------- 2. 移到上方，再检测一次 ----------------
x, y = float(top[0]), float(top[1])
move_to([x, y, 0.32], base_orn, steps=220)
info2 = detect_cube("hover")
if info2 is not None:
    x, y = float(info2["top"][0]), float(info2["top"][1])
    yaw_est = info2["yaw"]
    info["grasp_z"] = info2["grasp_z"]
    top = info2["top"]
    print("第二次感知: xy=(%.3f, %.3f) yaw=%.1f" % (x, y, degrees(yaw_est)))

# ---------------- 3. 转腕对准 yaw，下降、夹紧、抬起 ----------------
grasp_orn = p.getQuaternionFromEuler([0, pi, yaw_est])
print("转到 yaw=%.1f 度并对准..." % degrees(yaw_est))
move_to([x, y, 0.32], grasp_orn, steps=180)
print("下降...")
move_to([x, y, info["grasp_z"]], grasp_orn, steps=240)

print("闭合夹爪...")
set_gripper(True)
for _ in range(200):
    p.stepSimulation()
    time.sleep(1.0 / 240.0)

ok, per, total, n = is_grasped(cube)
print("触觉: 左指 %.1f N，右指 %.1f N，总 %.1f N，接触点 %d"
      % (per[finger_joints[0]], per[finger_joints[1]], total, n))
if not ok:
    print("触觉反馈：没有夹稳，放弃")
    set_gripper(False)
    cv2.destroyAllWindows()
    p.disconnect()
    raise SystemExit

print("抬起...")
move_to([x, y, 0.32], grasp_orn, steps=200)
for _ in range(240):
    p.stepSimulation()
    time.sleep(1.0 / 240.0)

per, total, n = tactile_read(cube)
ok2 = all(per[f] > 5.0 for f in finger_joints)
final_z = p.getBasePositionAndOrientation(cube)[0][2]
print("抬起后触觉: 左指 %.1f N，右指 %.1f N，总 %.1f N"
      % (per[finger_joints[0]], per[finger_joints[1]], total))
print("最终结果: %s | 方块高度 %.3f m"
      % ("成功" if (ok2 and final_z > 0.15) else "失败", final_z))

for _ in range(120):
    p.stepSimulation()
    time.sleep(1.0 / 240.0)
cv2.destroyAllWindows()
p.disconnect()
