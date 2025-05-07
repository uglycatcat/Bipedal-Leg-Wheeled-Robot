import pybullet as p
import pybullet_data
import time

# 连接到 PyBullet GUI
physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())

# 加载地板和设置重力
p.loadURDF("plane.urdf")
p.setGravity(0, 0, -9.81)

# 加载机器人模型
robot_id = p.loadURDF("tita/urdf/robot.urdf", basePosition=[0, 0, 1])

# 保持程序运行
while True:
    p.stepSimulation()
    time.sleep(1.0 / 240.0)