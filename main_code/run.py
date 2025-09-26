#官方库
import mujoco
import mujoco.viewer
import os
import numpy as np

#自己库
import inital
import main

#如此完美的模块化设计，什么叫可读性？！什么叫可维护性？！
if __name__ == "__main__":
    #初始化
    x=inital.inital()
    #运行
    main.main(x)

print("母鸡卡 结束！")