import mujoco as mj
import mujoco.viewer
import time
import glfw
import numpy as np
from scipy.spatial.transform import Rotation as R
from ControlGUI import RobotControlGUI
import MotionControl

class RobotController:
    
    def __init__(self, model_path):
        # 加载模型和场景
        self.model = mj.MjModel.from_xml_path(model_path)
        self.data = mj.MjData(self.model)
        
        # 初始化仿真状态
        mj.mj_forward(self.model, self.data)
        
        # 初始化观察器，使用与simulate命令类似的配置
        self.viewer = mujoco.viewer.launch_passive(
            self.model,
            self.data,
            show_left_ui=False,
        )
        # 开启GUI子线程
        RobotControlGUI.start()
        # 初始化base_link的欧拉角
        self.base_link_euler=[]
        # 记录上次渲染时间
        self.last_render_time = time.time()
        
    def transmit_sim_data(self):
        # 提取关节的 qpos 和 qvel（跳过 free joint 部分）
        qpos_data = self.data.qpos[7:15]  # 8 个关节的位置
        qvel_data = self.data.qvel[6:14]  # 8 个关节的速度

        # 将角速度值更新到对应位置
        qpos_data[3] = qvel_data[3]
        qpos_data[7] = qvel_data[7]
        
        # 合并数据
        processed_data = np.concatenate([qpos_data, self.base_link_euler])
        
        # 传递更新后的数据
        RobotControlGUI.receive_data(processed_data)
        
    def set_all_joint_control(self):
        quat = self.data.qpos[3:7]  # base_link 的四元数
        r = R.from_quat([quat[1], quat[2], quat[3], quat[0]])  # 注意 scipy 是 [x, y, z, w]
        self.base_link_euler = r.as_euler('xyz', degrees=False)
        kp=1
        ki=0.1
        kd=0.1
        last_error=self.base_link_euler[1]
        
        error=last_error
        self.data.ctrl[0]=0
        self.data.ctrl[1]=0
        self.data.ctrl[2]=0
        self.data.ctrl[3]=3
        self.data.ctrl[4]=0
        self.data.ctrl[5]=0
        self.data.ctrl[6]=0
        self.data.ctrl[7]=3
        
    def run(self):
        """主仿真循环"""
        try:
            # 设置仿真步长
            step_time = self.model.opt.timestep
            
            while self.viewer.is_running():
                
                # 记录循环开始时间
                loop_start = time.time()
                
                # 设置控制数据
                self.set_all_joint_control()
                
                # 步进仿真
                mj.mj_step(self.model, self.data)
                
                # 同步渲染，保持实时性
                now = time.time()
                elapsed = now - self.last_render_time
                if elapsed > 1.0/60.0:  # 约60Hz渲染频率
                    self.viewer.sync()
                    self.last_render_time = now
                    
                # 向GUI线程传递数据
                self.transmit_sim_data()
                
                # 计算并补偿仿真时间与实际时间的差异
                time_until_next_step = step_time - (time.time() - loop_start)
                if time_until_next_step > 0:
                    time.sleep(time_until_next_step)
                
        except KeyboardInterrupt:
            print("Simulation interrupted by user")
        finally:
            # 关闭GUI线程
            RobotControlGUI.stop()
            # 关闭窗口
            self.viewer.close()

if __name__ == "__main__":
    # 定义文件路径
    model_path = "tita/mjcf/scene.xml"
    
    # 创建控制器实例并运行仿真
    try:
        print("Starting simulation... (Press Ctrl+C to stop)")
        controller = RobotController(model_path)
        controller.run()
    except Exception as e:
        print(f"Error: {e}")
    finally:
        print("Simulation ended")