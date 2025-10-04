import mujoco
import numpy as np

xml = """
<mujoco>
  <option timestep="0.001" gravity="0 0 -9.81"/>
  <worldbody>
    <body name="box" pos="0 0 1">
      <joint type="free"/>
      <geom type="box" size="0.05 0.05 0.05" density="1000"/>
    </body>
  </worldbody>
</mujoco>
"""

m = mujoco.MjModel.from_xml_string(xml)
d = mujoco.MjData(m)
bid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "box")


def read_objacc(tag):
    # 确保派生量初始化
    mujoco.mj_forward(m, d)

    ares = np.zeros(6, dtype=np.float64)

    # 世界系（flg_local=0）
    mujoco.mj_objectAcceleration(m, d, mujoco.mjtObj.mjOBJ_BODY, bid, ares, 0)
    linacc_w = ares[3:6].copy()

    # 机体系（flg_local=1）
    mujoco.mj_objectAcceleration(m, d, mujoco.mjtObj.mjOBJ_BODY, bid, ares, 1)
    linacc_b = ares[3:6].copy()

    # 从姿态矩阵把世界系旋到机体系，做个交叉验证
    R_bw = d.xmat[bid].reshape(3, 3)  # body->world
    R_wb = R_bw.T
    linacc_b_from_w = R_wb @ linacc_w

    print(f"{tag} t={d.time:.4f} qz={d.qpos[2]:.4f} vz={d.qvel[2]:.4f}")
    print("  linacc_w_obj       :", linacc_w)
    print("  linacc_b_obj       :", linacc_b)
    print("  linacc_b_from_w    :", linacc_b_from_w)


# 初值读取
read_objacc("init")

# 有限差分（兜底对照）
vz_prev, t_prev = d.qvel[2], d.time
for i in range(5):
    mujoco.mj_step(m, d)  # 正常推进一步
    read_objacc(f"step {i + 1}")

    dt = d.time - t_prev
    a_fd_z = (d.qvel[2] - vz_prev) / dt if dt > 0 else np.nan
    print(f"  a_fd_z (finite diff) ≈ {a_fd_z:.6f}\n")
    vz_prev, t_prev = d.qvel[2], d.time
