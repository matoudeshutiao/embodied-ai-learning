import pybullet as p
import pybullet_data
import numpy as np
import cv2
import random
import time
from math import pi, tan, radians

# ============================================================
# 第六个练习：三色方块分类与分拣
#   固定俯视 RGB-D 相机 → HSV 颜色分类 → 抓取 → 放进同色盒子
#   盒子位置固定，方块位置随机；不需要 YOLO
# ============================================================

# ---------------- 0. 基础场景 ----------------
p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.8)
p.setRealTimeSimulation(0)
p.setTimeStep(1.0 / 240.0)
p.resetDebugVisualizerCamera(cameraDistance=1.5,
                             cameraYaw=50,
                             cameraPitch=-35,
                             cameraTargetPosition=[0.4, 0.0, 0.1])

p.loadURDF("plane.urdf")
panda = p.loadURDF("franka_panda/panda.urdf", useFixedBase=True)

CUBE_SCALE = 1.2
CUBE_HALF = 0.05 * CUBE_SCALE / 2.0   # 0.03m

COLORS = {
    "red":   [1.0, 0.0, 0.0, 1.0],
    "green": [0.0, 1.0, 0.0, 1.0],
    "blue":  [0.0, 0.0, 1.0, 1.0],
}
ORDER = ["red", "green", "blue"]

# 三个固定盒子：底部同色，位置固定（y=-0.20, 0, +0.20）
BIN_X = 0.55
BIN_Y = {"red": -0.20, "green": 0.0, "blue": 0.20}


def add_box(center, half, rgba):
    col = p.createCollisionShape(p.GEOM_BOX, halfExtents=half)
    vis = p.createVisualShape(p.GEOM_BOX, halfExtents=half, rgbaColor=rgba)
    p.createMultiBody(baseMass=0,
                      baseCollisionShapeIndex=col,
                      baseVisualShapeIndex=vis,
                      basePosition=center)


def make_bin(cx, cy, floor_color):
    """开口盒：彩色底板 + 四面灰色矮墙（墙高 4cm）"""
    add_box([cx, cy, 0.005], [0.06, 0.06, 0.005], floor_color)      # 底板
    wall = [0.6, 0.6, 0.6, 1.0]
    add_box([cx, cy - 0.055, 0.02], [0.06, 0.005, 0.02], wall)      # 前后
    add_box([cx, cy + 0.055, 0.02], [0.06, 0.005, 0.02], wall)
    add_box([cx - 0.055, cy, 0.02], [0.005, 0.06, 0.02], wall)      # 左右
    add_box([cx + 0.055, cy, 0.02], [0.005, 0.06, 0.02], wall)


for name in ORDER:
    make_bin(BIN_X, BIN_Y[name], COLORS[name])

# ---------------- 1. 随机放置三个方块 ----------------
positions = []
while len(positions) < 3:
    x = random.uniform(0.30, 0.45)
    y = random.uniform(-0.10, 0.10)
    if all((x - px) ** 2 + (y - py) ** 2 > 0.09 ** 2
           for px, py in positions):
        positions.append((x, y))

blocks = {}
for name, (x, y) in zip(ORDER, positions):
    b = p.loadURDF("cube_small.urdf", [x, y, 0.08], globalScaling=CUBE_SCALE)
    p.changeVisualShape(b, -1, rgbaColor=COLORS[name])
    blocks[name] = b
    print("方块 %-5s 初始位置 (%.3f, %.3f)" % (name, x, y))

# 等方块落地停稳
for step in range(3000):
    p.stepSimulation()
    moving = False
    for b in blocks.values():
        lin, ang = p.getBaseVelocity(b)
        if (lin[0] ** 2 + lin[1] ** 2 + lin[2] ** 2) ** 0.5 > 0.01 or \
           (ang[0] ** 2 + ang[1] ** 2 + ang[2] ** 2) ** 0.5 > 0.1:
            moving = True
            break
    if step > 60 and not moving:
        break

# ---------------- 2. 机械臂初始化 ----------------
default_pose = [0, -0.785, 0, -2.356, 0, 1.571, 0.785]
for i in range(7):
    p.resetJointState(panda, i, default_pose[i])

ee_idx = 11
finger_joints = [i for i in range(p.getNumJoints(panda))
                 if "finger" in p.getJointInfo(panda, i)[1].decode()]
target_orn = p.getQuaternionFromEuler([0, pi, 0])
SCAN_POSE = [0.20, -0.35, 0.40]   # 扫视位：让开摆放区，避免遮挡相机


def set_gripper(closed):
    for f in finger_joints:
        p.setJointMotorControl2(panda, f, p.POSITION_CONTROL,
                                0.0 if closed else 0.04, force=200)


def move_to(target_pos, steps=200):
    """每步重解 IK 的关节位置控制"""
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


set_gripper(False)
print("机械臂移动到扫视位...")
move_to(SCAN_POSE, steps=200)

# ---------------- 3. 固定俯视 RGB-D 相机 ----------------
WIDTH, HEIGHT = 640, 480
FOV = 60.0
NEAR, FAR = 0.05, 5.0
fx = (HEIGHT / 2.0) / tan(radians(FOV) / 2.0)
fy = fx
cx = WIDTH / 2.0
cy = HEIGHT / 2.0

CAM_EYE = [0.45, 0.0, 1.10]
CAM_TARGET = [0.45, 0.0, 0.0]
CAM_UP = [0.0, 1.0, 0.0]

HSV_RANGES = {
    "red":   [((0, 90, 60), (10, 255, 255)),
              ((170, 90, 60), (180, 255, 255))],
    "green": [((40, 80, 50), (85, 255, 255))],
    "blue":  [((100, 80, 50), (135, 255, 255))],
}
MASK_COLORS = {"red": (0, 0, 255), "green": (0, 255, 0), "blue": (255, 0, 0)}


def capture_top():
    """固定俯视相机：RGB + 米制深度"""
    view = p.computeViewMatrix(CAM_EYE, CAM_TARGET, CAM_UP)
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
    depth = FAR * NEAR / (FAR - (FAR - NEAR) * zbuf)
    return rgb, depth, zbuf


def color_mask(bgr, color):
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    mask = np.zeros(hsv.shape[:2], dtype=np.uint8)
    for lo, hi in HSV_RANGES[color]:
        mask |= cv2.inRange(hsv, lo, hi)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((5, 5), np.uint8))
    return mask


def points_from_mask(mask, depth, zbuf):
    """颜色掩码 + 深度 → 世界坐标点云"""
    vs, us = np.where((mask > 0) & (zbuf < 0.999))
    if len(us) == 0:
        return np.zeros((0, 3))
    Z = depth[vs, us]
    X = (us - cx) * Z / fx
    Y = -(vs - cy) * Z / fy

    forward = np.array(CAM_TARGET, dtype=float) - np.array(CAM_EYE, dtype=float)
    forward /= np.linalg.norm(forward)
    right = np.cross(forward, np.array(CAM_UP, dtype=float))
    right /= np.linalg.norm(right)
    cam_up = np.cross(right, forward)

    return (np.array(CAM_EYE, dtype=float)
            + right[None, :] * X[:, None]
            + cam_up[None, :] * Y[:, None]
            + forward[None, :] * Z[:, None])


def top_center_in_box(points, xmin, xmax, ymin, ymax, zmin, zmax):
    keep = ((points[:, 0] > xmin) & (points[:, 0] < xmax) &
            (points[:, 1] > ymin) & (points[:, 1] < ymax) &
            (points[:, 2] > zmin) & (points[:, 2] < zmax))
    pts = points[keep]
    if len(pts) < 30:
        return None
    mn = pts.min(axis=0)
    mx = pts.max(axis=0)
    return np.array([(mn[0] + mx[0]) / 2.0,
                     (mn[1] + mx[1]) / 2.0,
                     mx[2]])


def show_top_view(rgb, depth, zbuf, masks, detections, tag):
    vis = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR).copy()
    for color in ORDER:
        cnts, _ = cv2.findContours(masks[color], cv2.RETR_EXTERNAL,
                                   cv2.CHAIN_APPROX_SIMPLE)
        cv2.drawContours(vis, cnts, -1, MASK_COLORS[color], 2)
        if color in detections:
            t = detections[color]
            cv2.putText(vis, "%s (%.3f, %.3f)" % (color, t[0], t[1]),
                        (10, 30 + 22 * ORDER.index(color)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55,
                        MASK_COLORS[color], 2)

    valid = zbuf < 0.999
    if valid.any():
        dmin, dmax = np.percentile(depth[valid], [2, 98])
        norm = np.clip((depth - dmin) / (dmax - dmin + 1e-6), 0, 1)
    else:
        norm = np.zeros_like(depth)
    depth_vis = cv2.applyColorMap((norm * 255).astype(np.uint8),
                                  cv2.COLORMAP_JET)
    cv2.imshow("Top camera - RGB (%s)" % tag, vis)
    cv2.imshow("Top camera - Depth (%s)" % tag, depth_vis)
    cv2.waitKey(1)


def scan_blocks(show=True):
    """扫描摆放区，返回 {颜色: 顶面世界坐标}"""
    rgb, depth, zbuf = capture_top()
    bgr = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
    masks = {c: color_mask(bgr, c) for c in ORDER}
    detections = {}
    for color in ORDER:
        pts = points_from_mask(masks[color], depth, zbuf)
        top = top_center_in_box(pts, 0.25, 0.50, -0.15, 0.15, 0.02, 0.12)
        if top is not None:
            detections[color] = top
    if show:
        show_top_view(rgb, depth, zbuf, masks, detections, "scan")
    return detections


# ---------------- 4. 触觉反馈 ----------------
CONTACT_FORCE_MIN = 5.0


def tactile_read(block_id):
    contacts = p.getContactPoints(bodyA=panda, bodyB=block_id)
    per_finger = {f: 0.0 for f in finger_joints}
    count = 0
    for c in contacts:
        link = c[3]
        if link in per_finger:
            per_finger[link] += c[9]
            count += 1
    return per_finger, sum(per_finger.values()), count


def is_grasped(block_id):
    per_finger, total, count = tactile_read(block_id)
    both = all(per_finger[f] > CONTACT_FORCE_MIN for f in finger_joints)
    return both, per_finger, total, count


# ---------------- 5. 抓取与放置 ----------------
def pick_block(color):
    block_id = blocks[color]
    detections = scan_blocks(show=True)
    if color not in detections:
        print("扫描中没找到 %s 方块" % color)
        return False

    top = detections[color]
    x, y = top[0], top[1]
    grasp_z = float(np.clip(top[2] - CUBE_HALF + 0.0075, 0.03, 0.05))
    print("抓取 %s：目标 (%.3f, %.3f)，抓取高度 %.3f"
          % (color, x, y, grasp_z))

    move_to([x, y, 0.30], steps=220)
    move_to([x, y, grasp_z], steps=240)

    set_gripper(True)
    for _ in range(200):
        p.stepSimulation()
        time.sleep(1.0 / 240.0)

    both, per_finger, total, count = is_grasped(block_id)
    print("触觉：左指 %.1f N，右指 %.1f N，总 %.1f N，接触点 %d"
          % (per_finger[finger_joints[0]], per_finger[finger_joints[1]],
             total, count))
    if not both:
        print("触觉反馈：没有夹稳 %s，跳过" % color)
        set_gripper(False)
        move_to([x, y, 0.30], steps=160)
        return False

    # 抬起并持续监测，确认没有滑落
    print("抬起 %s..." % color)
    slipped = False
    for step in range(200):
        joint_pose = p.calculateInverseKinematics(
            panda, ee_idx, [x, y, 0.30], target_orn,
            maxNumIterations=200, residualThreshold=1e-8)
        for i in range(7):
            p.setJointMotorControl2(panda, i, p.POSITION_CONTROL,
                                    joint_pose[i], force=200)
        p.stepSimulation()
        time.sleep(1.0 / 240.0)
        if step % 20 == 0 and step > 20:
            still, _, _, _ = is_grasped(block_id)
            if not still:
                print("触觉反馈：抬升中丢失 %s" % color)
                slipped = True
                break

    if slipped:
        return False
    return True


def verify_in_bin(color):
    """用固定俯视相机确认方块在对应盒子里（靠高度过滤掉彩色底板）"""
    rgb, depth, zbuf = capture_top()
    bgr = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
    masks = {c: color_mask(bgr, c) for c in ORDER}
    pts = points_from_mask(masks[color], depth, zbuf)
    bx, by = BIN_X, BIN_Y[color]
    # z > 0.025：盒子底板在 z≈0.01，方块顶面在 z≈0.07
    top = top_center_in_box(pts, bx - 0.07, bx + 0.07,
                            by - 0.07, by + 0.07, 0.025, 0.15)
    detections = {color: top} if top is not None else {}
    show_top_view(rgb, depth, zbuf, masks, detections, "verify " + color)

    if top is None:
        print("验证：%s 盒子里没有检测到方块" % color)
        return False
    inside = (abs(top[0] - bx) < 0.07) and (abs(top[1] - by) < 0.07)
    print("验证：%s 方块估计位置 (%.3f, %.3f, %.3f) | 盒子中心 (%.3f, %.3f) | %s"
          % (color, top[0], top[1], top[2], bx, by,
             "在盒内" if inside else "不在盒内"))
    return inside


def place_in_bin(color):
    bx, by = BIN_X, BIN_Y[color]
    print("移动到 %s 盒子上方..." % color)
    move_to([bx, by, 0.30], steps=240)
    move_to([bx, by, 0.10], steps=200)

    print("松开夹爪，放下 %s..." % color)
    set_gripper(False)
    for _ in range(200):
        p.stepSimulation()
        time.sleep(1.0 / 240.0)

    _, total, count = tactile_read(blocks[color])
    print("放下确认：与方块的接触点 %d（0 表示已脱离）" % count)

    move_to([bx, by, 0.30], steps=160)
    print("回到扫视位验证...")
    move_to(SCAN_POSE, steps=240)
    for _ in range(200):
        p.stepSimulation()
        time.sleep(1.0 / 240.0)
    return verify_in_bin(color)


# ============================================================
# 6. 主流程：红 → 绿 → 蓝
# ============================================================
print("=" * 56)
print("  三色分拣开始：HSV 分类 + 固定俯视 RGB-D 相机")
print("=" * 56)

results = {}
for color in ORDER:
    print("\n----- 处理 %s -----" % color)
    if not pick_block(color):
        results[color] = False
        # 抓取失败时机械臂可能停在方块上方，先回扫视位
        move_to(SCAN_POSE, steps=220)
        continue
    results[color] = place_in_bin(color)

print("\n" + "=" * 56)
print("分拣结果：")
for color in ORDER:
    print("  %-5s -> %-5s 盒子：%s"
          % (color, color, "成功" if results.get(color) else "失败"))
print("=" * 56)

# 让窗口停留一会儿便于观察
for _ in range(240):
    p.stepSimulation()
    time.sleep(1.0 / 240.0)

cv2.destroyAllWindows()
p.disconnect()
