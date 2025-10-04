import numpy as np
import time

import inital
import main_step


def temp_start(mujoco_handler):
    '''
    定义初始化参数
    '''
    angle_deg = 30
    angle_rad = angle_deg * np.pi / 180.0
    v = 17.3

    vz = np.sin(angle_rad) * v
    vx = np.cos(angle_rad) * v
    vy = 0

    # 他妈的，搞了半天，这个API真是糖丸
    mujoco_handler.data.qvel[mujoco_handler.joint_id["missile_main_joint"]["qvel_adr"] + 0] = vx
    mujoco_handler.data.qvel[mujoco_handler.joint_id["missile_main_joint"]["qvel_adr"] + 1] = vy
    mujoco_handler.data.qvel[mujoco_handler.joint_id["missile_main_joint"]["qvel_adr"] + 2] = vz


# 如此完美的模块化设计，什么叫可读性？！什么叫可维护性？！
def main():
    
    # 初始化
    mujoco_handler = inital.inital()
    # 开局发射
    temp_start(mujoco_handler)
    # 持续运行
    time_stamp = 0

    while mujoco_handler.viewer.is_running():
        time.sleep(0.01)
        time_stamp += 1
        main_step.step(mujoco_handler, time_stamp)

    print("母鸡卡 结束！")


if __name__ == "__main__":
    main()
