import pybullet as p
import pybullet_data
import numpy as np
import cv2
import random
import time
from math import pi, tan, radians

# ============================================================
# 第五个练习：视觉伺服闭环
#   看见红色方块 → 用 RGB-D 算出它的三维位置 → 移过去 → 抓取
# 相机装在夹爪上（eye-in-hand），每一轮感知都会更新一次目标
# ============================================================

# ---------------- 0. 基础场景 ----------------
p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.8)
p.setRealTimeSimulation(0)
p.setTimeStep(1.0 / 240.0)
p.resetDebugVisualizerCamera(cameraDistance=1.3,
                             cameraYaw=45,
                             cameraPitch=-25,
                             cameraTargetPosition=[0.4, 0.0, 0.2])

p.loadURDF("plane.urdf")
panda = p.loadURDF("franka_panda/panda.urdf", useFixedBase=True)

# 方块随机放在相机观察范围内，每次运行位置不同
# 缩放 1.2：边长 6cm，夹爪总开度约 8cm，每边留 1cm 余量；
# 用 1.5 倍（7.5cm）时每边只剩 2.5mm，视觉定位的 1~2mm 误差就可能把方块顶歪。
CUBE_SCALE = 1.2
CUBE_HALF = 0.05 * CUBE_SCALE / 2.0   # 方块半边长 = 0.03m

cube_pos = [random.uniform(0.34, 0.48),
            random.uniform(-0.12, 0.12),
            0.08]
cube = p.loadURDF("cube_small.urdf", cube_pos, globalScaling=CUBE_SCALE)
p.changeVisualShape(cube, -1, rgbaColor=[1, 0, 0, 1])
print("方块初始位置:", [round(v, 3) for v in cube_pos])

# 等方块落地停稳
for step in range(3000):
    p.stepSimulation()
    lin, ang = p.getBaseVelocity(cube)
    speed = (lin[0] ** 2 + lin[1] ** 2 + lin[2] ** 2) ** 0.5
    spin = (ang[0] ** 2 + ang[1] ** 2 + ang[2] ** 2) ** 0.5
    if step > 60 and speed < 0.01 and spin < 0.1:
        break

# ---------------- 1. 机械臂初始化 ----------------
default_pose = [0, -0.785, 0, -2.356, 0, 1.571, 0.785]
for i in range(7):
    p.resetJointState(panda, i, default_pose[i])

ee_idx = 11
finger_joints = [i for i in range(p.getNumJoints(panda))
                 if "finger" in p.getJointInfo(panda, i)[1].decode()]
target_orn = p.getQuaternionFromEuler([0, pi, 0])

for f in finger_joints:
    p.setJointMotorControl2(panda, f, p.POSITION_CONTROL, 0.04, force=200)


def move_to(target_pos, steps=200):
    """每步重解 IK，把末端移动到 target_pos（和前面的项目一致）"""
    for _ in range(steps):
        joint_pose = p.calculateInverseKinematics(
            panda, ee_idx, target_pos, target_orn,
            maxNumIterations=200, residualThreshold=1e-8,
        )
        for i in range(7):
            p.setJointMotorControl2(panda, i, p.POSITION_CONTROL,
                                    joint_pose[i], force=200)
        p.stepSimulation()
        time.sleep(1.0 / 240.0)


# ---------------- 2. RGB-D 相机 ----------------
WIDTH, HEIGHT = 640, 480
FOV = 60.0
NEAR, FAR = 0.05, 5.0
fx = (HEIGHT / 2.0) / tan(radians(FOV) / 2.0)
fy = fx
cx = WIDTH / 2.0
cy = HEIGHT / 2.0


def get_camera_pose():
    """相机装在夹爪后上方，跟随机械臂移动"""
    hand = p.getLinkState(panda, ee_idx)[4]
    eye = [hand[0] - 0.12, hand[1], hand[2] + 0.12]
    target = [hand[0], hand[1], 0.0]
    up = [0.0, 1.0, 0.0]
    return eye, target, up


def capture_rgbd():
    eye, target, up = get_camera_pose()
    view = p.computeViewMatrix(eye, target, up)
    proj = p.computeProjectionMatrixFOV(FOV, WIDTH / HEIGHT, NEAR, FAR)
    try:
        _, _, rgb_pix, depth_pix, _ = p.getCameraImage(
            WIDTH, HEIGHT, viewMatrix=view, projectionMatrix=proj,
            renderer=p.ER_BULLET_HARDWARE_OPENGL,
        )
    except p.error:
        _, _, rgb_pix, depth_pix, _ = p.getCameraImage(
            WIDTH, HEIGHT, viewMatrix=view, projectionMatrix=proj,
            renderer=p.ER_TINY_RENDERER,
        )
    rgb = np.reshape(np.array(rgb_pix, dtype=np.uint8),
                     (HEIGHT, WIDTH, 4))[:, :, :3]
    zbuf = np.reshape(np.array(depth_pix, dtype=np.float32), (HEIGHT, WIDTH))
    # NDC 深度 → 米
    depth = FAR * NEAR / (FAR - (FAR - NEAR) * zbuf)
    return rgb, depth, zbuf, eye, target, up


def detect_red_cube(rgb, depth, zbuf, eye, target, up):
    """HSV 找红色 → 深度反投影 → 三维包围盒 → 顶面中心"""
    bgr = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, (0, 90, 60), (10, 255, 255))
    mask |= cv2.inRange(hsv, (170, 90, 60), (180, 255, 255))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((5, 5), np.uint8))

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL,
                                   cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return None, mask, bgr
    contour = max(contours, key=cv2.contourArea)
    if cv2.contourArea(contour) < 80:
        return None, mask, bgr

    clean = np.zeros_like(mask)
    cv2.drawContours(clean, [contour], -1, 255, -1)
    vs, us = np.where((clean > 0) & (zbuf < 0.999))
    if len(us) < 30:
        return None, clean, bgr

    Z = depth[vs, us]
    X = (us - cx) * Z / fx
    Y = -(vs - cy) * Z / fy

    forward = np.array(target, dtype=float) - np.array(eye, dtype=float)
    forward /= np.linalg.norm(forward)
    right = np.cross(forward, np.array(up, dtype=float))
    right /= np.linalg.norm(right)
    cam_up = np.cross(right, forward)

    points = (np.array(eye, dtype=float)
              + right[None, :] * X[:, None]
              + cam_up[None, :] * Y[:, None]
              + forward[None, :] * Z[:, None])
    keep = ((points[:, 0] > 0.15) & (points[:, 0] < 0.75) &
            (np.abs(points[:, 1]) < 0.35) &
            (points[:, 2] > 0.0) & (points[:, 2] < 0.40))
    points = points[keep]
    if len(points) < 30:
        return None, clean, bgr

    mn = points.min(axis=0)
    mx = points.max(axis=0)
    top_center = np.array([(mn[0] + mx[0]) / 2.0,
                           (mn[1] + mx[1]) / 2.0,
                           mx[2]])
    return {"top": top_center,
            "bbox": cv2.boundingRect(contour),
            "n": len(points)}, clean, bgr


def show_frame(bgr, depth, zbuf, mask, info, tag):
    """把检测结果显示到 OpenCV 窗口（标题和叠加文字用英文避免字体问题）"""
    vis = bgr.copy()
    if info is not None:
        x, y, w, h = info["bbox"]
        cv2.rectangle(vis, (x, y), (x + w, y + h), (0, 255, 0), 2)
        top = info["top"]
        cv2.putText(vis, "%s  top: %.3f %.3f %.3f m" % (tag, *top),
                    (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 0), 2)
    else:
        cv2.putText(vis, "%s  no red object" % tag,
                    (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)

    valid = zbuf < 0.999
    if valid.any():
        dmin, dmax = np.percentile(depth[valid], [2, 98])
        norm = np.clip((depth - dmin) / (dmax - dmin + 1e-6), 0, 1)
    else:
        norm = np.zeros_like(depth)
    depth_vis = cv2.applyColorMap((norm * 255).astype(np.uint8),
                                  cv2.COLORMAP_JET)
    cv2.drawContours(depth_vis, cv2.findContours(
        mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[0],
        -1, (0, 255, 0), 2)

    cv2.imshow("Visual servoing - RGB", vis)
    cv2.imshow("Visual servoing - Depth", depth_vis)
    cv2.waitKey(1)


def perceive(tag):
    rgb, depth, zbuf, eye, target, up = capture_rgbd()
    info, mask, bgr = detect_red_cube(rgb, depth, zbuf, eye, target, up)
    show_frame(bgr, depth, zbuf, mask, info, tag)
    return info


# ---------------- 3. 视觉伺服闭环 ----------------
LOOK_POSE = [0.40, 0.0, 0.42]   # 观察位
HOVER_Z = 0.30                  # 对准时的悬停高度

print("=" * 56)
print("  视觉伺服：看见 → 对准 → 下降 → 抓取")
print("=" * 56)

print("移动到观察位...")
move_to(LOOK_POSE, steps=200)

info = perceive("1. look")
if info is None:
    print("观察位没检测到红色方块，退出")
    cv2.destroyAllWindows()
    p.disconnect()
    raise SystemExit

# 两轮"感知 → 移动"迭代：第一轮把夹爪移到方块上方，第二轮微调
for attempt in range(2):
    top = info["top"]
    print("第 %d 次感知：估计顶面 (%.3f, %.3f, %.3f) m"
          % (attempt + 1, top[0], top[1], top[2]))
    move_to([top[0], top[1], HOVER_Z], steps=160)
    info = perceive("2. align" if attempt == 0 else "3. refine")
    if info is None:
        print("对准过程中丢失目标，退出")
        cv2.destroyAllWindows()
        p.disconnect()
        raise SystemExit

top = info["top"]
hand = p.getLinkState(panda, ee_idx)[4]
align_err = ((top[0] - hand[0]) ** 2 + (top[1] - hand[1]) ** 2) ** 0.5

truth = p.getBasePositionAndOrientation(cube)[0]
vision_err = ((top[0] - truth[0]) ** 2 + (top[1] - truth[1]) ** 2) ** 0.5
print("末端与目标水平偏差: %.4f m" % align_err)
print("视觉估计顶面: (%.3f, %.3f, %.3f) | 真值中心: (%.3f, %.3f, %.3f) | 水平误差: %.4f m"
      % (top[0], top[1], top[2], truth[0], truth[1], truth[2], vision_err))

# 由感知到的顶面高度推算指尖目标：方块中心上方 7.5mm
grasp_z = float(np.clip(top[2] - CUBE_HALF + 0.0075, 0.02, 0.06))
print("由视觉高度推出抓取高度: %.3f m" % grasp_z)

print("下降到抓取高度...")
move_to([hand[0], hand[1], grasp_z], steps=240)

descended = p.getLinkState(panda, ee_idx)[4]
cube_now = p.getBasePositionAndOrientation(cube)[0]
fingers_now = [round(p.getJointState(panda, f)[0], 4) for f in finger_joints]
print("下降后末端位置: (%.3f, %.3f, %.3f)，目标 z=%.3f"
      % (descended[0], descended[1], descended[2], grasp_z))
print("下降后方块位置: (%.3f, %.3f, %.3f)，手指 %s"
      % (cube_now[0], cube_now[1], cube_now[2], fingers_now))

print("闭合夹爪...")
for f in finger_joints:
    p.setJointMotorControl2(panda, f, p.POSITION_CONTROL, 0.0, force=200)
for _ in range(200):
    p.stepSimulation()
    time.sleep(1.0 / 240.0)

fingers_closed = [round(p.getJointState(panda, f)[0], 4) for f in finger_joints]
contacts = len(p.getContactPoints(bodyA=panda, bodyB=cube))
print("闭合后手指 %s，手指与方块接触点数 %d" % (fingers_closed, contacts))

print("抬起...")
move_to([hand[0], hand[1], grasp_z + 0.25], steps=200)

# 保持 1 秒并判断结果（仿真里可以用真值；真实系统要靠相机或触觉）
for _ in range(240):
    p.stepSimulation()
    time.sleep(1.0 / 240.0)

final_z = p.getBasePositionAndOrientation(cube)[0][2]
if final_z > 0.15:
    print("视觉伺服抓取成功，方块高度: %.3f m" % final_z)
else:
    print("视觉伺服抓取失败，方块高度: %.3f m" % final_z)

cv2.destroyAllWindows()
p.disconnect()
