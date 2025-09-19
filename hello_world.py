import mujoco
import mujoco.viewer
import os

mjcf_file_path = os.path.join(os.path.dirname(__file__), 'place.xml')
model = mujoco.MjModel.from_xml_path(mjcf_file_path)
data = mujoco.MjData(model)

print("MuJoCo model loaded successfully.")
print("Launching viewer with custom side orthographic view...")

with mujoco.viewer.launch_passive(model, data) as viewer:
    # 初始化 free camera
    mujoco.mjv_defaultFreeCamera(model, viewer.cam)

    # 手动设置自由相机参数
    viewer.cam.type = mujoco.mjtCamera.mjCAMERA_FREE
    viewer.cam.lookat[:] = [13, 0, 1.0]   # 场景中点，大概看中间（Z 调成 1 m 高度）
    viewer.cam.distance = 25              # 缩放，使目标和起点都进画面
    viewer.cam.azimuth = 90                # 90° 表示沿 Y 轴负方向→X 正方向看
    viewer.cam.elevation = 0               # 水平
    viewer.cam.orthographic = 1            # 正交投影

    while viewer.is_running():
        mujoco.mj_step(model, data)
        viewer.sync()