#官方库
import mujoco
import mujoco.viewer
import os
import numpy as np
import time
#自己库
import inital
import main_step

#如此完美的模块化设计，什么叫可读性？！什么叫可维护性？！
if __name__ == "__main__":
    #初始化
    x=inital.inital()
    #运行
    while x.viewer.is_running():
        time.sleep(0.01)
        main_step.step(x)

print("母鸡卡 结束！")