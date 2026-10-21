import pybullet as p
import pybullet_data
import time
from math import pi

# 1. 基础连接与物理设置
p.connect(p.GUI)
# 相机：distance 越小画面越大；target 是镜头中心点，这里对准机械臂工作区
p.resetDebugVisualizerCamera(cameraDistance=1.3,
                             cameraYaw=45,
                             cameraPitch=-22,
                             cameraTargetPosition=[0.25, 0.0, 0.25])
# 内置模型（plane.urdf、cube_small.urdf、franka_panda/panda.urdf）都在 pybullet_data 里
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -1.6)
p.setRealTimeSimulation(0)

# 2. 加载地面和Franka机械臂
p.loadURDF("plane.urdf")
panda = p.loadURDF("franka_panda/panda.urdf", useFixedBase=True)

# 3. 在机械臂前方放置一个红色木块
block_pos = [0.4, 0.0, 5]
block_id = p.loadURDF("cube_small.urdf", block_pos, globalScaling=1.5)
p.changeVisualShape(block_id, -1, rgbaColor=[1, 0, 0, 1])

# 等待木块落地并停稳（替代原来写死的 300 步）
#
# 为什么不能写死步数：
#   自由落体时间 t = sqrt(2h/g)，需要的步数 ≈ t × 240（仿真频率 240Hz）。
#   之前 g=1.6、木块从 z=5 落到 z=0.0375，h=4.9625m，
#   t = sqrt(2×4.9625/1.6) ≈ 2.49s → 约 600 步；
#   而原来只跑 300 步，木块才落下 1.25m，还悬在 z≈3.75m 的空中。
#   脚本这时就去读木块位置算抓取目标（约 3.9m 高），远超机械臂 0.855m 的臂展，
#   IK 不会报错而是返回"最接近的解"，机械臂就会猛拉到关节限位，看起来像失控。
# 所以这里改成检测"速度是否足够小"：不管重力、高度、物体怎么改，
#   只要木块真的停稳了才继续，步数由物理过程自己决定。
SETTLE_TIMEOUT = 3000  # 安全上限：3000 步 = 12.5 秒仿真时间，防止死等
for step in range(SETTLE_TIMEOUT):
    p.stepSimulation()
    time.sleep(1.0 / 240.0)  # 实时播放；想让它算快点（不观看）可以删掉这行

    lin, ang = p.getBaseVelocity(block_id)
    speed = (lin[0] ** 2 + lin[1] ** 2 + lin[2] ** 2) ** 0.5   # 线速度 m/s
    spin = (ang[0] ** 2 + ang[1] ** 2 + ang[2] ** 2) ** 0.5    # 角速度 rad/s

    if step % 60 == 0:  # 每 60 步（0.25 秒）打印一次高度，观察下落过程
        print("木块高度: %.2f m" % p.getBasePositionAndOrientation(block_id)[0][2])

    # step > 60：跳过刚生成时的初始状态，避免刚开始就被误判为"已经静止"
    # 速度阈值 0.01 m/s、0.1 rad/s 是经验值：再往下看就是肉眼静止了
    if step > 60 and speed < 0.01 and spin < 0.1:
        print("木块在第 %d 步停稳" % step)
        break
else:
    print("警告：等待超时，木块可能还没停稳，后面的抓取可能失败")

# 4. 机械臂初始姿态设置
default_pose = [0, -0.785, 0, -2.356, 0, 1.571, 0.785]
for i in range(7):
    p.resetJointState(panda, i, default_pose[i])

# 5. 识别末端执行器和手指关节
ee_idx = 11
finger_joints = []
for i in range(p.getNumJoints(panda)):
    joint_name = p.getJointInfo(panda, i)[1].decode()
    if "finger" in joint_name:
        finger_joints.append(i)

# 6. 设置夹爪初始张开
for f in finger_joints:
    p.setJointMotorControl2(panda, f, p.POSITION_CONTROL, 0.04, force=200)

# 7. 运动到目标点：每走一步都重新解一次 IK，种子用当前关节角，比只解一次稳定得多
target_orn = p.getQuaternionFromEuler([0, pi, 0])  # 夹爪朝下


def move_to(target_pos, steps):
    for _ in range(steps):
        joint_pose = p.calculateInverseKinematics(
            panda, ee_idx, target_pos, target_orn,
            maxNumIterations=200, residualThreshold=1e-8,
        )
        for i in range(7):
            p.setJointMotorControl2(panda, i, p.POSITION_CONTROL, joint_pose[i], force=200)
        p.stepSimulation()
        time.sleep(1.0 / 240.0)


block_xyz = p.getBasePositionAndOrientation(block_id)[0]
grasp_pos = [block_xyz[0], block_xyz[1], block_xyz[2] + 0.0075]  # 指尖对准木块中心
above_pos = [block_xyz[0], block_xyz[1], block_xyz[2] + 0.16]    # 木块正上方

# 8. 分阶段动作：接近 → 下降 → 闭合 → 抬起
print("移动到木块上方...")
move_to(above_pos, 240)

print("下降到木块...")
move_to(grasp_pos, 240)

print("闭合夹爪...")
for f in finger_joints:
    p.setJointMotorControl2(panda, f, p.POSITION_CONTROL, 0.0, force=200)
for _ in range(200):
    p.stepSimulation()
    time.sleep(1.0 / 240.0)

print("抬起木块...")
move_to([block_xyz[0], block_xyz[1], block_xyz[2] + 0.26], 240)

# 保持抓到手的姿态 1 秒，方便观察
for _ in range(240):
    p.stepSimulation()
    time.sleep(1.0 / 240.0)

lifted = p.getBasePositionAndOrientation(block_id)[0]
print("抓取完成，木块最终高度: %.3f m" % lifted[2])
p.disconnect()
