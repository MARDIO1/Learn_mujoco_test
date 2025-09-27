import mujoco
import mujoco.viewer
import os
import numpy as np
#自己的库
import inital
def step(x):
    
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