#官方库
import mujoco
import mujoco.viewer
import os
import numpy as np
import time
#自己库
import inital
import main_step

def temp_start(x):
   
    angle_degrees = 30
    angle_rad = angle_degrees * np.pi / 180.0
    v=17
    vz=np.sin(angle_rad)*v
    vx=np.cos(angle_rad)*v
    vy=0
    x.data.qvel[x.joint_id['missile_main_joint']['qvel_adr']+0]=vx#他妈的，搞了半天，这个API真是糖丸
    x.data.qvel[x.joint_id['missile_main_joint']['qvel_adr']+1]=vy
    x.data.qvel[x.joint_id['missile_main_joint']['qvel_adr']+2]=vz
    return 0

#如此完美的模块化设计，什么叫可读性？！什么叫可维护性？！
if __name__ == "__main__":
    #初始化
    x=inital.inital()
    #开局发射
    temp_start(x)
    #持续运行
    time_stamp=0
    while x.viewer.is_running():
        
        time.sleep(0.01)
        time_stamp+=1
        #if time_stamp<1000:
        #    x.data.xfrc_applied[6, 2] = 10.0  # 1 N
        main_step.step(x,time_stamp)
print("母鸡卡 结束！")