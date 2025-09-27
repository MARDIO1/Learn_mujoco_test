import mujoco
import mujoco.viewer
import os
import numpy as np
#各项参数用来快速调整
xml_file_path=r'..\mode\all.xml'



#封装了一个类
class Option:
    def __init__(self,path):
        #xml文件参数#一些大接口参数
        self.mjcf_file_path = os.path.join(os.path.dirname(__file__), path)
        self.model = mujoco.MjModel.from_xml_path(self.mjcf_file_path)
        self.data = mujoco.MjData(self.model)
        self.viewer = None  # 修改为 self.viewer = None
        #一些自定义小接口参数
        self.body_id={}
        
    #显示画面大函数
    def launch_viewer(self):
        """启动 viewer 窗口"""
        if self.viewer is None:
            self.viewer = mujoco.viewer.launch_passive(self.model, self.data)
            mujoco.mjv_defaultFreeCamera(self.model, self.viewer.cam)

            # 手动设置自由相机参数
            self.viewer.cam.type = mujoco.mjtCamera.mjCAMERA_FREE
            self.viewer.cam.lookat[:] = [13, 0, 1.0]   # 场景中点，大概看中间（Z 调成 1 m 高度）
            self.viewer.cam.distance = 40              # 缩放，使目标和起点都进画面
            self.viewer.cam.azimuth = 90                # 90° 表示沿 Y 轴负方向→X 正方向看
            self.viewer.cam.elevation = 0               # 水平
            self.viewer.cam.orthographic = 1            # 正交投影
    
    #id对应名字,构造函数的一部分
    def get_all_body_ids(self):
        print("总共有",self.model.nbody,"个body")
        for i in range(self.model.nbody):  # model.nbody 是模型中 body 的总数
            body_name = self.model.body(i).name # 获取 body 的名称，注意新版本
            if body_name:  # 确保 body 有名称 (有些 body 可能没有名称)
                self.body_id[body_name] = i  # 将名称和 ID 添加到字典中
            print(i," ",body_name)
        return 0

def inital():
    #加载文件，初始化各项参数
    print("母鸡卡 启动！")
    option=Option(xml_file_path)
    option.get_all_body_ids()
    print("模型初始化 胜利！")
    print("初始化 free camera")
    option.launch_viewer()
    print("free camera 初始化 胜利！")
    return option

if __name__ == "__main__":
    pass