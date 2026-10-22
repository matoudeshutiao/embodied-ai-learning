import pybullet as p
import pybullet_data
import time
from math import pi

# ============================================================
# 0. 基础环境（和第二个项目相同）
# ============================================================
p.connect(p.GUI)
p.resetDebugVisualizerCamera(cameraDistance=1.3,
                             cameraYaw=45,
                             cameraPitch=-22,
                             cameraTargetPosition=[0.25, 0.0, 0.25])
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.8)
p.setRealTimeSimulation(0)

p.loadURDF("plane.urdf")
panda = p.loadURDF("franka_panda/panda.urdf", useFixedBase=True)

# ============================================================
# 1. 放木块，等它落地停稳
# ============================================================
block_pos = [0.4, 0.0, 0.05]
block_id = p.loadURDF("cube_small.urdf", block_pos, globalScaling=1.5)
p.changeVisualShape(block_id, -1, rgbaColor=[1, 0, 0, 1])

# 用速度判断是否停稳，而不是写死步数（原因见第二个项目的注释）
for step in range(3000):
    p.stepSimulation()
    lin, ang = p.getBaseVelocity(block_id)
    speed = (lin[0] ** 2 + lin[1] ** 2 + lin[2] ** 2) ** 0.5
    spin = (ang[0] ** 2 + ang[1] ** 2 + ang[2] ** 2) ** 0.5
    if step > 60 and speed < 0.01 and spin < 0.1:
        break

# ============================================================
# 2. 机械臂初始化
# ============================================================
default_pose = [0, -0.785, 0, -2.356, 0, 1.571, 0.785]
for i in range(7):
    p.resetJointState(panda, i, default_pose[i])

# 末端参考点：11 = panda_grasptarget（两指之间的抓取中心）
ee_idx = 11

# 夹爪手指关节
finger_joints = []
for i in range(p.getNumJoints(panda)):
    joint_name = p.getJointInfo(panda, i)[1].decode()
    if "finger" in joint_name:
        finger_joints.append(i)

# 夹爪张开
for f in finger_joints:
    p.setJointMotorControl2(panda, f, p.POSITION_CONTROL, 0.04, force=200)

# 夹爪始终朝下
target_orn = p.getQuaternionFromEuler([0, pi, 0])


def report(tag):
    """打印当前末端、木块和手指状态，方便观察每个阶段的结果"""
    ee = p.getLinkState(panda, ee_idx)[4]
    blk = p.getBasePositionAndOrientation(block_id)[0]
    fin = [round(p.getJointState(panda, f)[0], 4) for f in finger_joints]
    print("%s | 末端 (%.3f, %.3f, %.3f) | 木块 z=%.3f | 手指 %s"
          % (tag, ee[0], ee[1], ee[2], blk[2], fin))


# ============================================================
# 3. 四个阶段函数：每个函数只负责一个动作
# ============================================================
def move_to(target_pos, steps=200):
    """把末端移动到 target_pos：每一步都重新解一次 IK

    为什么要每步重解，而不是只解一次：
    pybullet 的 calculateInverseKinematics 是局部数值解，单次求解可能停在
    离目标几厘米的位置。实测从"上方"姿态直接解抓取点，末端只降到 z=0.073
    （目标 0.045），夹爪会在木块顶面上方闭合，抓空；把上一次的解作为新种子
    再解，误差会从 2.9cm 收敛到 2.5mm。每走一步都用当前关节角当种子重解，
    等价于不断重新播种，最终能稳定到达目标。
    """
    for _ in range(steps):
        # IK：把"末端要到的位置"换算成"7 个关节的角度"
        joint_pose = p.calculateInverseKinematics(
            panda, ee_idx, target_pos, target_orn,
            maxNumIterations=200, residualThreshold=1e-8,
        )
        # 位置控制：给每个关节一个目标角度，电机自己转过去
        for i in range(7):
            p.setJointMotorControl2(panda, i, p.POSITION_CONTROL, joint_pose[i], force=200)
        # 推进仿真一步（240Hz，200 步 ≈ 0.83 秒）
        p.stepSimulation()
        time.sleep(1.0 / 240.0)


def move_above(obj_pos, offset_z=0.15):
    """阶段 1：先到物体正上方，避免斜着撞过去"""
    target_pos = [obj_pos[0], obj_pos[1], obj_pos[2] + offset_z]
    move_to(target_pos)


def move_down(block_id, grasp_z, tol=0.005, steps=200):
    """阶段 2：带感知的连续下探：边走边确认木块还在不在

    这里把"运动"和"感知"解耦：

      - 运动：末端连续地朝 grasp_z 移动，每一步重解 IK，和普通 move_to 一样平滑；
      - 感知：同样每一步检查一次木块位置，发现偏移超过 tol 立即中止。

    为什么不再按 1cm 分段：
      分段会让机械臂"走一步、停一下"，15 段 × 40 步 = 600 步（约 2.5 秒）。
      连续版本 200 步（约 0.83 秒）就能走完，感知频率反而更高（每步都查）。

    参数：
      block_id：木块刚体编号
      grasp_z ：最终抓取高度（世界坐标的绝对高度）
      tol     ：允许的木块水平偏移，超过就中止，默认 5mm
      steps   ：下探阶段最多推进多少仿真步，默认 200（约 0.83 秒）
    返回：
      True  = 到达抓取高度
      False = 木块移动，或末端没有收敛到目标
    """
    start_xyz = p.getBasePositionAndOrientation(block_id)[0]
    target_pos = [start_xyz[0], start_xyz[1], grasp_z]
    print("开始带感知下探，木块初始位置:", [round(v, 4) for v in start_xyz])

    for step_i in range(steps):
        # 1.【感知】木块相对下探开始时移动了多少（只看水平方向）
        block_xyz = p.getBasePositionAndOrientation(block_id)[0]
        drift = ((block_xyz[0] - start_xyz[0]) ** 2 +
                 (block_xyz[1] - start_xyz[1]) ** 2) ** 0.5
        if drift > tol:
            print("感知到木块偏移 %.3f m，超过阈值 %.3f m，中止下探" % (drift, tol))
            return False

        # 2.【决策】木块位置没变，继续朝固定的抓取点下降

        # 3.【执行】IK + 位置控制，走一小步
        joint_pose = p.calculateInverseKinematics(
            panda, ee_idx, target_pos, target_orn,
            maxNumIterations=200, residualThreshold=1e-8,
        )
        for i in range(7):
            p.setJointMotorControl2(panda, i, p.POSITION_CONTROL, joint_pose[i], force=200)
        p.stepSimulation()
        time.sleep(1.0 / 240.0)

        if step_i % 40 == 0:  # 每 1/6 秒打印一次，避免刷屏
            ee = p.getLinkState(panda, ee_idx)[4]
            print("下探中 z=%.3f（木块偏移 %.4f m）" % (ee[2], drift))

    ee = p.getLinkState(panda, ee_idx)[4]
    err = abs(ee[2] - grasp_z)
    print("下探结束 z=%.3f，末端误差 %.4f m" % (ee[2], err))
    return err < 0.005


def grasp():
    """阶段 3：闭合夹爪"""
    # 目标 0.0 = 两指完全闭合；实际会在夹住木块时被阻挡，形成夹持力
    for f in finger_joints:
        p.setJointMotorControl2(panda, f, p.POSITION_CONTROL, 0.0, force=200)
    for _ in range(200):
        p.stepSimulation()
        time.sleep(1.0 / 240.0)


def lift(height=0.3):
    """阶段 4：保持夹紧，垂直抬起

    注意：height 也是绝对高度。
    """
    current_pos = p.getLinkState(panda, ee_idx)[4]
    target_pos = [current_pos[0], current_pos[1], height]
    move_to(target_pos)


# ============================================================
# 4. 组合调用：像搭积木一样拼出完整抓取流程
# ============================================================
block_xyz = p.getBasePositionAndOrientation(block_id)[0]
print("木块停稳位置:", [round(v, 4) for v in block_xyz])

print("阶段 1：移动到木块正上方")
move_above(block_xyz, offset_z=0.15)
report("上方")

print("阶段 2：带感知地下探到抓取高度")
grasped = move_down(block_id, grasp_z=block_xyz[2] + 0.0075)
if grasped:
    report("下降后")

    print("阶段 3：闭合夹爪")
    grasp()
    report("闭合后")

    print("阶段 4：抬起")
    lift(height=block_xyz[2] + 0.26)
    report("抬起后")
else:
    print("下探失败：木块移动或末端未到位，取消抓取")

# 保持 1 秒，方便观察
for _ in range(240):
    p.stepSimulation()
    time.sleep(1.0 / 240.0)

lifted = p.getBasePositionAndOrientation(block_id)[0]
if grasped:
    print("抓取完成，木块最终高度: %.3f m" % lifted[2])
else:
    print("未抓取，木块最终高度: %.3f m" % lifted[2])
p.disconnect()
