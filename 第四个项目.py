import pybullet as p
import pybullet_data
import numpy as np
import cv2
import time
from math import pi, tan, radians

# ============================================================
# 第四个练习：给机械臂装上 RGB-D 相机
# 相机固定在夹爪上方，跟着机械臂一起移动（eye-in-hand）
# 实时显示 RGB、深度图，并用颜色 + 深度定位红色方块的三维位置
# ============================================================

# ---------------- 0. 基础场景 ----------------
p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.8)
p.setRealTimeSimulation(0)
p.setTimeStep(1.0 / 240.0)
p.resetDebugVisualizerCamera(cameraDistance=1.25,
                             cameraYaw=45,
                             cameraPitch=-25,
                             cameraTargetPosition=[0.35, 0.0, 0.2])

p.loadURDF("plane.urdf")
panda = p.loadURDF("franka_panda/panda.urdf", useFixedBase=True)

# 红色方块（边长 0.075m，静止时中心高度 0.0375m，顶面 0.075m）
cube = p.loadURDF("cube_small.urdf", [0.4, 0.0, 0.05], globalScaling=1.5)
p.changeVisualShape(cube, -1, rgbaColor=[1, 0, 0, 1])

# 等方块落地
for _ in range(300):
    p.stepSimulation()

# ---------------- 1. 机械臂移动到观察位 ----------------
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
    """每步重解 IK，把末端移动到 target_pos（和第三个项目一致）"""
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


print("移动到观察位...")
move_to([0.4, 0.0, 0.40], steps=200)

# ============================================================
# 2. RGB-D 相机参数
# ============================================================
WIDTH, HEIGHT = 640, 480
FOV = 60.0          # 垂直视场角（度）
NEAR, FAR = 0.05, 5.0

# 相机内参：由垂直 FOV 和图像高度算出焦距（像素单位）
fx = (HEIGHT / 2.0) / tan(radians(FOV) / 2.0)
fy = fx
cx = WIDTH / 2.0
cy = HEIGHT / 2.0

CUBE_HALF = 0.0375  # 仅用于把顶面高度换算回方块中心做验证


def get_camera_pose():
    """相机安装位置：夹爪上方 12cm、向后 12cm，镜头朝下看向桌面

    每一帧都从 ee_idx 的当前世界位姿算出来，所以机械臂移动时相机跟着走。
    真实系统里这个"手眼标定"关系需要事先标定；仿真里直接由几何关系给出。
    """
    hand = p.getLinkState(panda, ee_idx)[4]
    eye = [hand[0] - 0.12, hand[1], hand[2] + 0.12]
    target = [hand[0], hand[1], 0.0]   # 看向夹爪正下方的桌面
    up = [0.0, 1.0, 0.0]               # 俯视时用一个不与视线平行的方向作为"上"
    return eye, target, up


def capture_rgbd():
    """渲染 RGB + 深度，并把 PyBullet 的非线性深度缓冲换算成米"""
    eye, target, up = get_camera_pose()
    view = p.computeViewMatrix(eye, target, up)
    proj = p.computeProjectionMatrixFOV(FOV, WIDTH / HEIGHT, NEAR, FAR)

    try:
        _, _, rgb_pix, depth_pix, _ = p.getCameraImage(
            WIDTH, HEIGHT, viewMatrix=view, projectionMatrix=proj,
            renderer=p.ER_BULLET_HARDWARE_OPENGL,
        )
    except p.error:
        # 没有硬件 OpenGL 时退回到 CPU 渲染
        _, _, rgb_pix, depth_pix, _ = p.getCameraImage(
            WIDTH, HEIGHT, viewMatrix=view, projectionMatrix=proj,
            renderer=p.ER_TINY_RENDERER,
        )

    rgb = np.reshape(np.array(rgb_pix, dtype=np.uint8),
                     (HEIGHT, WIDTH, 4))[:, :, :3]
    zbuf = np.reshape(np.array(depth_pix, dtype=np.float32), (HEIGHT, WIDTH))

    # PyBullet 的深度缓冲是 NDC 值（0 近，1 远），不是米：
    #   z_ndc = (far + near)/(far - near) / 2 - far*near/(z * (far-near)) ...
    # 反解得到真实深度（沿相机光轴的距离）：
    depth = FAR * NEAR / (FAR - (FAR - NEAR) * zbuf)
    return rgb, depth, zbuf, eye, target, up


def detect_red_cube(rgb, depth, zbuf, eye, target, up):
    """颜色分割找红色像素 → 用深度反投影成三维点云 → 估计方块顶面中心"""
    bgr = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)

    # 红色在 HSV 里横跨 0 度，需要两段
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

    # 只保留最大轮廓，去掉零散噪点
    clean = np.zeros_like(mask)
    cv2.drawContours(clean, [contour], -1, 255, -1)
    vs, us = np.where((clean > 0) & (zbuf < 0.999))
    if len(us) < 30:
        return None, clean, bgr

    # 像素 → 相机坐标（Z 为正的深度，x 向右，y 向上）
    Z = depth[vs, us]
    X = (us - cx) * Z / fx
    Y = -(vs - cy) * Z / fy

    # 相机坐标系 → 世界坐标系
    # 相机光轴 forward 由 eye→target 决定；right/up 由 up 向量正交化得到
    forward = np.array(target, dtype=float) - np.array(eye, dtype=float)
    forward /= np.linalg.norm(forward)
    right = np.cross(forward, np.array(up, dtype=float))
    right /= np.linalg.norm(right)
    cam_up = np.cross(right, forward)

    points = (np.array(eye, dtype=float)
              + right[None, :] * X[:, None]
              + cam_up[None, :] * Y[:, None]
              + forward[None, :] * Z[:, None])

    # 只保留桌面附近的点，过滤背景
    keep = ((points[:, 0] > 0.15) & (points[:, 0] < 0.75) &
            (np.abs(points[:, 1]) < 0.35) &
            (points[:, 2] > 0.0) & (points[:, 2] < 0.40))
    points = points[keep]
    if len(points) < 30:
        return None, clean, bgr

    # 可见表面的三维包围盒；顶面中心 = (x 中点, y 中点, z 最大值)
    mn = points.min(axis=0)
    mx = points.max(axis=0)
    top_center = np.array([(mn[0] + mx[0]) / 2.0,
                           (mn[1] + mx[1]) / 2.0,
                           mx[2]])
    bbox = cv2.boundingRect(contour)
    return {"top": top_center, "bbox": bbox, "n": len(points)}, clean, bgr


# ============================================================
# 3. 实时显示 RGB + 深度，并验证三维定位
# ============================================================
print("=" * 56)
print("  RGB-D 相机已启动：'RGB-D camera - RGB' / '- Depth'")
print("  在 OpenCV 窗口里按 q 退出；关闭 PyBullet 窗口也可以")
print("  控制台会打印估计的方块顶面位置，并与仿真真值对比")
print("=" * 56)

frame = 0
try:
    while p.isConnected():
        rgb, depth, zbuf, eye, target, up = capture_rgbd()
        info, mask, bgr = detect_red_cube(rgb, depth, zbuf, eye, target, up)

        if info is not None:
            x, y, w, h = info["bbox"]
            cv2.rectangle(bgr, (x, y), (x + w, y + h), (0, 255, 0), 2)
            top = info["top"]
            cv2.putText(bgr, "top: %.3f %.3f %.3f m" % tuple(top),
                        (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

            if frame % 30 == 0:
                truth = p.getBasePositionAndOrientation(cube)[0]
                err_xy = ((top[0] - truth[0]) ** 2 +
                          (top[1] - truth[1]) ** 2) ** 0.5
                print("估计顶面 (%.3f, %.3f, %.3f) | "
                      "真值中心 (%.3f, %.3f, %.3f) | 水平误差 %.3f m"
                      % (top[0], top[1], top[2],
                         truth[0], truth[1], truth[2], err_xy))
        else:
            cv2.putText(bgr, "no red object", (10, 25),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

        # 深度图伪彩色显示：按有效像素的范围自动归一化
        valid = zbuf < 0.999
        if valid.any():
            dmin, dmax = np.percentile(depth[valid], [2, 98])
            depth_norm = np.clip((depth - dmin) / (dmax - dmin + 1e-6), 0, 1)
        else:
            depth_norm = np.zeros_like(depth)
        depth_vis = cv2.applyColorMap((depth_norm * 255).astype(np.uint8),
                                      cv2.COLORMAP_JET)
        cv2.drawContours(depth_vis, cv2.findContours(
            mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[0], -1,
            (0, 255, 0), 2)

        cv2.imshow("RGB-D camera - RGB", bgr)
        cv2.imshow("RGB-D camera - Depth", depth_vis)

        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break
        try:
            if cv2.getWindowProperty("RGB-D camera - RGB",
                                     cv2.WND_PROP_VISIBLE) < 1:
                break
        except cv2.error:
            break

        frame += 1
        p.stepSimulation()
        time.sleep(0.01)

except p.error:
    print("PyBullet 窗口已关闭，退出")
finally:
    cv2.destroyAllWindows()
