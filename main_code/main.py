import mujoco
import mujoco.viewer
import os
import numpy as np

import inital
def main(x):
    while x.viewer.is_running():
        mujoco.mj_step(x.model, x.data)
        x.viewer.sync()
if __name__ == "__main__":
    pass