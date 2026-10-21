import pybullet as p
import pybullet_data
import time
from math import pi

# 连接GUI窗口
p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())    
p.setAdditionalSearchPath(r"C:\Users\28400\Desktop\具身学习")
p.setGravity(0, 0, -9.8)
p.setRealTimeSimulation(0)

# 加载地面和Panda机械臂
p.loadURDF("plane.urdf")
panda = p.loadURDF("franka_panda/panda.urdf", useFixedBase=True)

# 将机械臂复位到默认位形
default_pose = [0, -0.785, 0, -2.356, 0, 1.571, 0.785]
for i in range(7):
    p.resetJointState(panda, i, default_pose[i])

# 打印所有关节名，方便查索引
for i in range(p.getNumJoints(panda)):
    print(i, p.getJointInfo(panda, i)[1])

# 末端执行器link索引
ee_idx = 11

# 找出夹爪手指关节
finger_joints = []
for i in range(p.getNumJoints(panda)):
    joint_name = p.getJointInfo(panda, i)[1].decode()
    if "finger" in joint_name:
        finger_joints.append(i)
print("手指关节:", finger_joints)

# 夹爪张开
for f in finger_joints:
    p.setJointMotorControl2(panda, f, p.POSITION_CONTROL, 0.04)

# IK逆解：让末端移动到物体上方位置
target_pos = [0.4, 0.0, 0.5]
target_orn = p.getQuaternionFromEuler([0, pi, 0])
joint_pose = p.calculateInverseKinematics(panda, ee_idx, target_pos, target_orn,
                                          maxNumIterations=200, residualThreshold=1e-8)

# 控制前7个关节运动
for i in range(7):
    p.setJointMotorControl2(panda, i, p.POSITION_CONTROL, joint_pose[i])

# 运行仿真循环
for _ in range(500):
    p.stepSimulation()
    time.sleep(1.0 / 240.0)