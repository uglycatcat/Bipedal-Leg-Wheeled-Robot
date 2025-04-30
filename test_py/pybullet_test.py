import pybullet as p
import pybullet_data
import time

physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
robot_id = p.loadURDF("tita/urdf/robot.urdf")

# 保持程序运行
while True:
    p.stepSimulation()
    time.sleep(1.0 / 240.0)  # 设置模拟步长