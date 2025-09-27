import mujoco
import mujoco.viewer
import os
import numpy as np
#自己的库
import inital
def step(x):
    while x.viewer.is_running():
        mujoco.mj_step(x.model, x.data)#模拟器运行
        x.viewer.sync()#画面显示
    return 0
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

if __name__ == "__main__":
    pass