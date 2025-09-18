import mujoco
import mujoco.viewer  # 需要单独安装 mujoco-python-viewer 才能用
import os

mjcf_file_path = os.path.join(os.path.dirname(__file__), 'place.xml')

model = mujoco.MjModel.from_xml_path(mjcf_file_path)
data = mujoco.MjData(model)

print("MuJoCo model loaded successfully.")
print("Launching interactive viewer... (Press ESC to close)")

# 启动交互式仿真窗口
with mujoco.viewer.launch_passive(model, data) as viewer:
    while viewer.is_running():
        mujoco.mj_step(model, data)
        viewer.sync()