import mujoco
import numpy as np
import math
import rerun as rr
# 自己的库
import my_math
import my_parameter

pose_data = my_parameter.PoseData()
missile_parameter = my_parameter.MissileParameter()
power_data = my_parameter.PowerData()


def pose_get(x, body_id):
    """void mj_objectAcceleration(const mjModel* m, const mjData* d,
    int objtype, int objid, mjtNum res[6], int flg_local);"""
    a_res = np.zeros(6, dtype=np.float64)
    mujoco.mj_objectAcceleration(
        x.model,  # 模型
        x.data,  # 数据
        mujoco.mjtObj.mjOBJ_BODY,  # 对象类型为BODY
        body_id,  # 身体ID
        a_res,  # 结果存储数组
        0,  # flg_local设为1，表示使用局部坐标系
    )
    """print("a_res",a_res)"""
    # 初始化 aoa_degrees 和 sideslip_angle_degrees，确保它们总有值
    aoa_degrees = 0.0
    sideslip_angle_degrees = 0.0
    aoa = 0.0
    sideslip_angle = 0.0

    # 获取并归一化四元数（MuJoCo顺序为[w,x,y,z]）
    q = my_math.normalize_quat(x.data.body(body_id).xquat)
    yaw_pitch_roll = my_math.get_euler_angles(q)
    # 获取Body的质心线速度（世界坐标系对齐）
    v_global = x.data.body(body_id).cvel[3:6].copy()
    # 获取从世界到body局部坐标系的旋转矩阵，怎么感觉不对啊啊啊啊
    R_world_to_body = x.data.body(body_id).xmat.reshape(3, 3)
    # 全局速度获取
    w_local = x.data.body(body_id).cvel[0:3].copy()  # 1-3是体坐标系角速度
    v_global = (
        x.data.body(body_id).cvel[3:6].copy()
    )  # 3-6是世界线速度 呜呜呜，唐没变了呜呜呜呜呜呜呜呜呜
    # 将世界坐标系下的速度转换到Body局部坐标系下
    v_local = v_global @ R_world_to_body

    # v_local = my_math.quat_rotate_vector(q, v_global)
    pose_data.v_wind_mps = np.linalg.norm(v_global)  # 计算速度向量的模
    if pose_data.v_wind_mps < 0.5:
        aoa = 0.0  # 或 np.nan，取决于你的需求
        sideslip_angle = 0.0  # 或 np.nan
    else:
        # 只有当速度大于阈值时才计算
        aoa = np.arctan2(v_local[2], v_local[0])
        sideslip_angle = np.arctan2(v_local[1], v_local[0])
        aoa_degrees = np.degrees(aoa)
        sideslip_angle_degrees = np.degrees(sideslip_angle)

    # 构造风轴系,用弹体系速度定义风轴，得到风->体旋转矩阵
    v_body = np.array(pose_data.v_local_mps, dtype=np.float64)
    R_bw, _ = my_math._wind_axes_in_body(v_body)  # 风轴系到弹体系的旋转矩阵

    # print(v_local)

    """v_local已经验证，正确完了给你
    [ 0.8660254 -0.        -0.5      ]
    [ 0.         1.        -0.       ]
    [ 0.5        0.         0.8660254]
    风轴系好像也正确
    [[-0.9999995   0.          0.00100007]
    [-0.         -1.          0.        ]
    [ 0.00100007  0.          0.9999995 ]]
    """

    pose_data.R_wind_to_local = R_bw
    pose_data.R_local_to_global = R_world_to_body
    pose_data.q_wind_to_global = my_math.q_wind_to_world(q, v_body)
    # pose_data.v_global_mps=
    pose_data.q_local_to_global = q  # mujoco提取姿态矩阵 世界->弹体
    pose_data.v_local_mps = v_local
    pose_data.v_global_mps = v_global
    pose_data.a_local_mps2 = x.data.body(body_id).cacc[3:6].copy()
    pose_data.b_local_radps2 = x.data.body(body_id).cacc[0:3].copy()

    pose_data.w_local_radps = w_local
    pose_data.aoa_rad = aoa
    pose_data.aoa_degree = aoa_degrees
    pose_data.soa_rad = sideslip_angle
    pose_data.soa_degree = sideslip_angle_degrees
    pose_data.yaw_pitch_roll_d = yaw_pitch_roll

    dt = x.model.opt.timestep  # 获取时间步长
    pose_data.aoadot_radps = (pose_data.aoa_rad - pose_data.last_aoa_rad) / dt
    pose_data.soadot_radps = (pose_data.soa_rad - pose_data.last_soa_rad) / dt
    pose_data.last_aoa_rad = pose_data.aoa_rad
    pose_data.last_soa_rad = pose_data.soa_rad
    return 0


# 逐帧计算空气动力
def air_power_cal_step():
    """
    计算导弹在空气动力作用下的力和力矩。

    Args:
        pose_data (Pose_data): 包含当前姿态和速度信息的对象。
        missile_parameter (MissileParameter): 包含导弹气动参数的对象。

    Returns:
        tuple: 包含两部分：
            - list: 本体参考系下的空气动力向量 [Fx, Fy, Fz] (单位: N)。
            - list: 本体参考系下的空气力矩向量 [Mx, My, Mz] (单位: Nm)。
    """
    rho = 1.225  # 空气密度，单位 kg/m^3
    # 从 pose_data 对象中解包局部速度和角速度
    vx_local, vy_local, vz_local = pose_data.v_local_mps
    wx_local, wy_local, wz_local = pose_data.w_local_radps  # p, q, r

    # 使用 Pose_data 对象中的迎角和侧滑角
    alpha = pose_data.aoa_rad
    beta = pose_data.soa_rad

    # 舵面偏转值 (假设为0，需要从控制系统获取)
    delta_e = pose_data.duo_y_rad
    delta_a = pose_data.duo_z_rad
    delta_r = 0.0

    # 从 missile_parameter 对象中获取参考面积和参考长度
    S_ref = missile_parameter.S_ref
    L_ref = missile_parameter.L_ref

    # 空速计算
    V_squared = vx_local**2 + vy_local**2 + vz_local**2
    if V_squared == 0:
        return [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]
    V = math.sqrt(V_squared)

    # 升力系数 CL
    # 检查 alpha 和 beta 的单位是否与系数定义一致（通常是弧度）
    CL = (
        missile_parameter.CL0
        + missile_parameter.CL_alpha * alpha
        + missile_parameter.CL_beta * beta
        + missile_parameter.CL_q * (wy_local * L_ref / (2 * V))  # q -> wy_local
        + missile_parameter.CL_p * (wx_local * L_ref / (2 * V))  # p -> wx_local
        + missile_parameter.CL_r * (wz_local * L_ref / (2 * V))  # r -> wz_local
        + missile_parameter.CL_de * delta_e
        + missile_parameter.CL_da * delta_a
        + missile_parameter.CL_dr * delta_r
    )

    # 阻力系数 CD
    CD = missile_parameter.CD0
    """ + 
          missile_parameter.CD_alpha * alpha + missile_parameter.CD_beta * beta +
          missile_parameter.CD_q * (wy_local * L_ref / (2 * V)) +
          missile_parameter.CD_p * (wx_local * L_ref / (2 * V)) +
          missile_parameter.CD_r * (wz_local * L_ref / (2 * V)) +
          missile_parameter.CD_de * delta_e + missile_parameter.CD_da * delta_a + missile_parameter.CD_dr * delta_r
          ) """
    # 计算CD各项贡献（便于定位）

    # 侧向力系数 CY
    CY = (
        missile_parameter.CY0
        + missile_parameter.CY_alpha * alpha
        + missile_parameter.CY_beta * beta
        + missile_parameter.CY_q * (wy_local * L_ref / (2 * V))
        + missile_parameter.CY_p * (wx_local * L_ref / (2 * V))
        + missile_parameter.CY_r * (wz_local * L_ref / (2 * V))
        + missile_parameter.CY_de * delta_e
        + missile_parameter.CY_da * delta_a
        + missile_parameter.CY_dr * delta_r
    )

    # 俯仰力矩系数 Cm
    Cm = (
        missile_parameter.Cm0
        + missile_parameter.Cm_alpha * alpha
        + missile_parameter.Cm_beta * beta
        + missile_parameter.Cm_q * (wy_local * L_ref / (2 * V))
        + missile_parameter.Cm_alphadot * (pose_data.aoadot_radps * L_ref / (2 * V))
        + missile_parameter.Cm_de * delta_e
        + missile_parameter.Cm_da * delta_a
        + missile_parameter.Cm_dr * delta_r
    )

    # 滚转力矩系数 Cl
    Cl = (
        missile_parameter.Cl0
        + missile_parameter.Cl_alpha * alpha
        + missile_parameter.Cl_beta * beta
        + missile_parameter.Cl_p * (wx_local * L_ref / (2 * V))
        + missile_parameter.Cl_r * (wz_local * L_ref / (2 * V))
        + missile_parameter.Cl_de * delta_e
        + missile_parameter.Cl_da * delta_a
        + missile_parameter.Cl_dr * delta_r
    )

    # 偏航力矩系数 Cn
    Cn = (
        missile_parameter.Cn0
        + missile_parameter.Cn_alpha * alpha
        + missile_parameter.Cn_beta * beta
        + missile_parameter.Cn_p * (wx_local * L_ref / (2 * V))
        + missile_parameter.Cn_r * (wz_local * L_ref / (2 * V))
        + missile_parameter.Cn_de * delta_e
        + missile_parameter.Cn_da * delta_a
        + missile_parameter.Cn_dr * delta_r
    )

    # 计算力和力矩 (单位: N, Nm)
    # 力
    D = 0.5 * rho * V_squared * S_ref * CD
    Y = 0.5 * rho * V_squared * S_ref * CY
    L = 0.5 * rho * V_squared * S_ref * CL

    # 力矩
    M = 0.5 * rho * V_squared * S_ref * L_ref * Cm  # 俯仰力矩
    Roll_Moment = 0.5 * rho * V_squared * S_ref * L_ref * Cl  # 滚转力矩
    N = 0.5 * rho * V_squared * S_ref * L_ref * Cn  # 偏航力矩

    # 风轴系中构造力（Xw沿气流方向：阻力D为正；Yw侧向；Zw升力）
    F_wind = np.array([D, Y, L], dtype=np.float64)
    power_data.F_wind = F_wind

    # 风->体->世界 最终转化为世界系
    power_data.F_local = pose_data.R_wind_to_local @ F_wind  # 中间件
    F_world = my_math.body_to_world(
        power_data.F_local, q_wb=pose_data.q_local_to_global
    )
    power_data.F_global = F_world.tolist()

    # 转化为世界系力矩
    power_data.M_local = np.array([Roll_Moment, M, N], dtype=np.float64)
    power_data.M_global = my_math.body_to_world(
        power_data.M_local, q_wb=pose_data.q_local_to_global
    )
    power_data.M_global = [Roll_Moment, M, N]

    # print(f"V={V:.3f}, CD={CD:.4f}, D={D:.2f}, v_body={v_body}, F_body={power_data.F_local}, dot={np.dot(power_data.F_local, v_body):.3e}")
    if pose_data.v_wind_mps < 1:  # 低速风轴乱动最唐解决方案，更唐的是，这还没解决的了
        power_data.M_global = [0, 0, 0]
        power_data.F_global = [0, 0, 0]
    return 0


def air_power_use_step(x, body_id):
    force_in_body_frame = np.array(power_data.F_global, dtype=np.float64)
    # force_in_body_frame = np.array([1,0,0], dtype=np.float64)
    torque_in_body_frame = np.array(power_data.M_global, dtype=np.float64)
    # print(f"在 body {body_id} 的世界坐标系下施加力: {force_in_body_frame}")
    # print(f"在 body {body_id} 的世界坐标系下施加力矩: {torque_in_body_frame}")

    # 要先清零！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！
    x.data.qfrc_applied[:] = 0.0
    mujoco.mj_applyFT(
        x.model,
        x.data,
        force_in_body_frame,  # 力的大小
        torque_in_body_frame,  # 扭矩
        x.data.xipos[body_id],  # 应用力的位置，这个参数是什么？代表joint？还是质心？
        body_id,  # 物体ID
        x.data.qfrc_applied,  # 应用力的数组，这个参数是什么？额外力？
    )


""" osc = debug.Oscilloscope(
    titles=["aoa", "F_wind", "F_wind"],
    colors=["C0", "C1", "C2"],
    time_window=0.5,  # 最近5秒滚动
    ylabel="degree"
    )
def debug_step(x,body_id,time_stamp):
    #osc.push(time_stamp*x.model.opt.timestep, [pose_data.aoa_degree, pose_data.soa_degree, power_data.F_wind[0]])
    osc.push(time_stamp*x.model.opt.timestep, [pose_data.aoa_degree, 0, 0])
    osc.update()  # 非阻塞刷新 """


def debug_step(x, body_id, time_stamp):
    print("风轴坐标系(相对弹体系)", "\n", pose_data.R_wind_to_local)
    print("世界坐标系下阻力矩阵", "\n", power_data.F_global)
    print("弹体坐标系速度", "\n", pose_data.v_local_mps)
    print("世界系速度", "\n", pose_data.v_global_mps)
    print("总共外加力", "\n", x.data.qfrc_applied)
    print("\n")

def rr_debug_step(x, body_id, time_stamp):
    rr.set_time("time_stamp", sequence=time_stamp)
    rr.log("AOA", rr.Scalars(pose_data.aoa_degree))
    rr.log("SOA", rr.Scalars(pose_data.soa_degree))

def step(x, time_stamp):
    body_id = x.model.body("missile").id
    pose_get(x, body_id)
    air_power_cal_step()
    air_power_use_step(x, body_id)
    debug_step(x, body_id, time_stamp)
    rr_debug_step(x, body_id, time_stamp)
    mujoco.mj_step(x.model, x.data)  # 模拟器运行
    x.viewer.sync()  # 更新画面显示


if __name__ == "__main__":
    pass
