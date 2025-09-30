import mujoco
import numpy as np
#自己的库
import inital
import my_math
class Pose_data:
    def __init__(self):
        #通常值
        self.yaw_pitch_woll={0,0,0}
        self.v_global={0,0,0}
        self.v_local={0,0,0}
        self.a={0,0,0}
        self.w={0,0,0}
        #特殊值
        self.aoa=0
        self.β=0

pose_data=Pose_data()
def pose_get(x,body_id):
    # 获取并归一化四元数（MuJoCo顺序为[w,x,y,z]）
    q = my_math.normalize_quat(x.data.body(body_id).xquat)
    yaw_pitch_roll=my_math.get_euler_angles(q)
    # 全局速度→体坐标系速度 
    w_global = x.data.body(body_id).cvel[0:3].copy()#1-3是体坐标系角速度
    v_global = x.data.body(body_id).cvel[3:6].copy()#3-6是线速度
    v_local = my_math.quat_rotate_vector(q, v_global)
    
    velocity_magnitude = np.linalg.norm(v_local) # 计算速度向量的模

    if velocity_magnitude < 0.5:
        aoa = 0.0  # 或 np.nan，取决于你的需求
        sideslip_angle = 0.0 # 或 np.nan
    else:
        # 只有当速度大于阈值时才计算
        aoa = np.arctan2(v_local[2], v_local[0])
        sideslip_angle = np.arctan2(v_local[1], v_local[0])
        aoa_degrees = np.degrees(aoa)
        sideslip_angle_degrees = np.degrees(sideslip_angle)
        print(aoa_degrees)
    #赋值
    #angle_global = x.data.body(body_id).cpos.copy()
    #print(yaw_pitch_roll)
    #print(aoa)
    pose_data.v_global=v_global
    pose_data.v_local=v_local
    pose_data.a=x.data.body(body_id).cacc[3:6].copy()
    pose_data.w=w_global
    pose_data.aoa=aoa
    return 0

def step(x):
    body_id = x.model.body("missile").id
    pose_get(x,body_id)
    mujoco.mj_step(x.model, x.data)#模拟器运行
    x.viewer.sync()#画面显示

#逐帧计算空气动力
'''
注意飞行力学公式
质心运动：
F=ma
旋转运动：
M=Ib
'''

def air_power_step(x):
    return 0
# qvel 数组排列规则针对 freejoint:
# data.qvel[qvel_start_index]        -> 线性速度 vx
# data.qvel[qvel_start_index + 1]    -> 线性速度 vy
# data.qvel[qvel_start_index + 2]    -> 线性速度 vz
# data.qvel[qvel_start_index + 3]    -> 角速度 wx (绕body局部x轴)
# data.qvel[qvel_start_index + 4]    -> 角速度 wy (绕body局部y轴)
# data.qvel[qvel_start_index + 5]    -> 角速度 wz (绕body局部z轴)
if __name__ == "__main__":
    pass