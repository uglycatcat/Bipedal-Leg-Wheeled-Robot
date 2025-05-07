import numpy as np
import mujoco
import mujoco.viewer
import glfw
import time
import math
from pathlib import Path

class RobotController:
    
    def __init__(self, model_path):
        # 加载三维模型
        self.model = mujoco.MjModel.from_xml_path(model_path)
        self.data = mujoco.MjData(self.model)
        
        # 初始化观察器
        self.viewer = mujoco.viewer.launch_passive(self.model, self.data)
        
        # 初始化时间跟踪
        self.last_update = time.time()
        
    def disable_mujoco_keys(self, window, key, scancode, action, mods):
        # 必要情况下屏蔽mujoco默认快捷键
        pass
        
    def run(self):
        """主仿真循环"""
        try:
            while self.viewer.is_running():
                # 步进仿真
                mujoco.mj_step(self.model, self.data)
                
                # 控制渲染频率
                current_time = time.time()
                if (current_time - self.last_update) > 0.02:
                    self.viewer.sync()
                    self.last_update = current_time
                
        except KeyboardInterrupt:
            pass
        finally:
            self.viewer.close()

if __name__ == "__main__":
    # 定义文件路径
    model_path = "tita/mjcf/scene.xml"  # Adjusted to match your file structure
    
    # 创造机器人控制实例并且运行模拟
    try:
        controller = RobotController(model_path)
        controller.run()
    except Exception as e:
        print(f"Error: {e}")