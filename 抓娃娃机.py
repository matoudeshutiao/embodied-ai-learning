import pybullet as p
import pybullet_data
import time
import random
from math import pi

# ============================================================
# 抓娃娃机：用键盘遥控 Panda 机械臂去夹橙色小方块
# 本地娱乐脚本，不提交到 GitHub
# ============================================================
p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.8)
p.setRealTimeSimulation(0)
p.setTimeStep(1.0 / 240.0)
p.resetDebugVisualizerCamera(cameraDistance=1.2,
                             cameraYaw=45,
                             cameraPitch=-25,
                             cameraTargetPosition=[0.4, 0.0, 0.18])

p.loadURDF("plane.urdf")
panda = p.loadURDF("franka_panda/panda.urdf", useFixedBase=True)

# 娃娃：橙色小方块（缩放 1.5 后边长 7.5cm，和之前抓取测试一致）
prize = p.loadURDF("cube_small.urdf", [0.5, 0.1, 0.06], globalScaling=1.5)
p.changeVisualShape(prize, -1, rgbaColor=[1.0, 0.55, 0.1, 1])

# ============================================================
# 机械臂初始化
# ============================================================
default_pose = [0, -0.785, 0, -2.356, 0, 1.571, 0.785]
for i in range(7):
    p.resetJointState(panda, i, default_pose[i])

ee_idx = 11
finger_joints = [i for i in range(p.getNumJoints(panda))
                 if "finger" in p.getJointInfo(panda, i)[1].decode()]
target_orn = p.getQuaternionFromEuler([0, pi, 0])

grip_closed = False


def set_gripper(closed):
    for f in finger_joints:
        p.setJointMotorControl2(panda, f, p.POSITION_CONTROL,
                                0.0 if closed else 0.04, force=200)


set_gripper(False)

# ============================================================
# 控制参数
# ============================================================
target = [0.45, 0.0, 0.30]          # 末端目标点，初始在木块上方
STEP_XY = 0.003                     # 每帧水平移动 3mm ≈ 0.72 m/s
STEP_Z = 0.0025                     # 每帧垂直移动 2.5mm
LIMIT_X = (0.22, 0.68)
LIMIT_Y = (-0.25, 0.25)
LIMIT_Z = (0.05, 0.45)

pressed = set()                     # 当前按住的键
grabbed = False                     # 是否已经夹起娃娃
prev_space = False                  # 上一帧空格是否按下（用于检测"按下沿"）
prev_reset = False                  # 上一帧 R 是否按下


def reset_prize():
    """重新随机放一个娃娃"""
    x = random.uniform(0.35, 0.60)
    y = random.uniform(-0.20, 0.20)
    yaw = random.uniform(-pi, pi)
    p.resetBasePositionAndOrientation(
        prize, [x, y, 0.08], p.getQuaternionFromEuler([0, 0, yaw]))
    p.resetBaseVelocity(prize, [0, 0, 0], [0, 0, 0])
    set_gripper(False)
    print("新娃娃放在 x=%.2f, y=%.2f" % (x, y))


print("=" * 52)
print("  抓娃娃机（先点击 PyBullet 窗口让它获得焦点）")
print("  W / ↑      : 向前       S / ↓   : 向后")
print("  A / ←      : 向左       D / →   : 向右")
print("  E / PgUp   : 上升       Q / PgDn: 下降")
print("  空格       : 夹紧 / 松开")
print("  R          : 重新放一个娃娃")
print("  Ctrl+C     : 退出")
print("=" * 52)

# ============================================================
# 主循环：每帧读键盘 → 更新目标 → IK → 电机 → 推进一步
# ============================================================
while p.isConnected():
    keys = p.getKeyboardEvents()

    # 维护"按住"集合；松开时移除（比只看 KEY_IS_DOWN 更稳）
    for code, state in keys.items():
        if state & (p.KEY_WAS_TRIGGERED | p.KEY_IS_DOWN):
            pressed.add(code)
        if state & p.KEY_WAS_RELEASED:
            pressed.discard(code)

    # 方向键 / WASD 控制前后左右，PgUp/PgDn 控制升降
    dx = dy = dz = 0.0
    if (p.B3G_UP_ARROW in pressed) or (ord('w') in pressed):
        dx += STEP_XY
    if (p.B3G_DOWN_ARROW in pressed) or (ord('s') in pressed):
        dx -= STEP_XY
    if (p.B3G_LEFT_ARROW in pressed) or (ord('a') in pressed):
        dy += STEP_XY
    if (p.B3G_RIGHT_ARROW in pressed) or (ord('d') in pressed):
        dy -= STEP_XY
    if (p.B3G_PAGE_UP in pressed) or (ord('e') in pressed):
        dz += STEP_Z
    if (p.B3G_PAGE_DOWN in pressed) or (ord('q') in pressed):
        dz -= STEP_Z

    # 限制在机械臂比较稳的可达范围内
    target[0] = min(max(target[0] + dx, LIMIT_X[0]), LIMIT_X[1])
    target[1] = min(max(target[1] + dy, LIMIT_Y[0]), LIMIT_Y[1])
    target[2] = min(max(target[2] + dz, LIMIT_Z[0]), LIMIT_Z[1])

    # 空格：切换夹爪；R：重放娃娃
    # 用"按下沿"判断，避免长按或系统自动重复导致开关反复触发
    space_down = p.B3G_SPACE in pressed
    if space_down and not prev_space:
        grip_closed = not grip_closed
        set_gripper(grip_closed)
        print("夹爪:", "夹紧" if grip_closed else "张开")
    prev_space = space_down

    reset_down = ord('r') in pressed
    if reset_down and not prev_reset:
        reset_prize()
    prev_reset = reset_down

    # 每帧重新解 IK，让末端跟随键盘目标
    # 窗口被关掉时物理连接会断开，捕获后干净退出，避免打印 traceback
    try:
        joint_pose = p.calculateInverseKinematics(
            panda, ee_idx, target, target_orn,
            maxNumIterations=100, residualThreshold=1e-6,
        )
        for i in range(7):
            p.setJointMotorControl2(panda, i, p.POSITION_CONTROL,
                                    joint_pose[i], force=200)

        p.stepSimulation()
    except p.error:
        print("窗口已关闭，退出")
        break

    # 抓到提示：夹紧且娃娃被抬到一定高度
    prize_z = p.getBasePositionAndOrientation(prize)[0][2]
    if grip_closed and prize_z > 0.15 and not grabbed:
        grabbed = True
        print("抓到娃娃啦！按空格松开夹爪")
    if not grip_closed and grabbed and prize_z < 0.06:
        grabbed = False

    time.sleep(1.0 / 240.0)
