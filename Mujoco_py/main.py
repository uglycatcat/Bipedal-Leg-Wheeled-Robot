import mujoco as mj
import mujoco.viewer
import time
import glfw
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
        # 记录上次渲染时间
        self.last_render_time = time.time()
        
    def run(self):
        """主仿真循环"""
        try:
            # 设置仿真步长
            step_time = self.model.opt.timestep
            
            while self.viewer.is_running():
                # 记录循环开始时间
                loop_start = time.time()
                
                # 步进仿真
                mj.mj_step(self.model, self.data)
                
                # 同步渲染，保持实时性
                now = time.time()
                elapsed = now - self.last_render_time
                if elapsed > 1.0/60.0:  # 约60Hz渲染频率
                    self.viewer.sync()
                    self.last_render_time = now
                # 传递数据
                RobotControlGUI.receive_data(self.data.actuator_length[:8])
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