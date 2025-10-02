import mujoco
import numpy as np
import math
#自己的库
import inital
import my_math
import my_parameter 

pose_data=my_parameter.Pose_data()
missile_parameter = my_parameter.MissileParameter()
power_data=my_parameter.Power_data()
def pose_get(x,body_id):
    '''void mj_objectAcceleration(const mjModel* m, const mjData* d,
                           int objtype, int objid, mjtNum res[6], int flg_local);'''
    a_res = np.zeros(6, dtype=np.float64)
    mujoco.mj_objectAcceleration(
        x.model,  # 模型
        x.data,   # 数据
        mujoco.mjtObj.mjOBJ_BODY,  # 对象类型为BODY
        body_id,  # 身体ID
        a_res,      # 结果存储数组
        0        # flg_local设为1，表示使用局部坐标系
    )
    '''print("a_res",a_res)'''
    # 初始化 aoa_degrees 和 sideslip_angle_degrees，确保它们总有值
    aoa_degrees = 0.0
    sideslip_angle_degrees = 0.0
    aoa = 0.0
    sideslip_angle = 0.0

    # 获取并归一化四元数（MuJoCo顺序为[w,x,y,z]）
    q = my_math.normalize_quat(x.data.body(body_id).xquat)
    yaw_pitch_roll=my_math.get_euler_angles(q)
    # 全局速度→体坐标系速度 
    w_local = x.data.body(body_id).cvel[0:3].copy()#1-3是体坐标系角速度
    v_local = x.data.body(body_id).cvel[3:6].copy()#3-6是线速度
    #v_local = my_math.quat_rotate_vector(q, v_global)
    velocity_magnitude = np.linalg.norm(v_local) # 计算速度向量的模
    if velocity_magnitude < 0.5:
        aoa = 0.0  # 或 np.nan，取决于你的需求
        sideslip_angle = 0.0 # 或 np.nan
    else:
        # 只有当速度大于阈值时才计算
        aoa = np.arctan2(v_local[2], v_local[0])
        sideslip_angle = np.arctan2(v_local[1], v_local[0])
        aoa_degrees = np.degrees(aoa)
        sideslip_angle_degrees = np.degrees(sideslip_angle)
        #print(aoa_degrees)

    #pose_data.v_global_mps=v_global
    pose_data.v_local_mps=v_local
    pose_data.a_local_mps2=x.data.body(body_id).cacc[3:6].copy()
    pose_data.b_local_radps2=x.data.body(body_id).cacc[0:3].copy()

    pose_data.w_local_radps=w_local
    pose_data.aoa_rad=aoa
    pose_data.aoa_degree=aoa_degrees
    pose_data.soa_rad=sideslip_angle
    pose_data.soa_degree=sideslip_angle_degrees
    pose_data.yaw_pitch_roll_d=yaw_pitch_roll
    
    dt = x.model.opt.timestep # 获取时间步长
    pose_data.aoadot_radps=(pose_data.aoa_rad-pose_data.last_aoa_rad)/dt
    pose_data.soadot_radps=(pose_data.soa_rad-pose_data.last_soa_rad)/dt
    pose_data.last_aoa_rad=pose_data.aoa_rad
    pose_data.last_soa_rad=pose_data.soa_rad
    return 0

#逐帧计算空气动力
'''
注意飞行力学公式
质心运动：
F=ma
旋转运动：
M=Ib

动力学公式：
F_lip=
F_slide=
F_gravity=mg
F_drag=

各个方向力矩

'''

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
    rho = 1.225 # 空气密度，单位 kg/m^3
    # 从 pose_data 对象中解包局部速度和角速度
    vx_local, vy_local, vz_local = pose_data.v_local_mps
    wx_local, wy_local, wz_local = pose_data.w_local_radps # p, q, r

    # 使用 Pose_data 对象中的迎角和侧滑角
    alpha = pose_data.aoa_rad
    beta = pose_data.soa_rad 

    # 舵面偏转值 (假设为0，需要从控制系统获取)
    delta_e =pose_data.duo_y_rad
    delta_a =pose_data.duo_z_rad
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
    CL = (missile_parameter.CL0 + missile_parameter.CL_alpha * alpha + missile_parameter.CL_beta * beta +
          missile_parameter.CL_q * (wy_local * L_ref / (2 * V)) + # q -> wy_local
          missile_parameter.CL_p * (wx_local * L_ref / (2 * V)) + # p -> wx_local
          missile_parameter.CL_r * (wz_local * L_ref / (2 * V)) + # r -> wz_local
          missile_parameter.CL_de * delta_e + missile_parameter.CL_da * delta_a + missile_parameter.CL_dr * delta_r)

    # 阻力系数 CD
    CD = (missile_parameter.CD0 + missile_parameter.CD_alpha * alpha + missile_parameter.CD_beta * beta +
          missile_parameter.CD_q * (wy_local * L_ref / (2 * V)) +
          missile_parameter.CD_p * (wx_local * L_ref / (2 * V)) +
          missile_parameter.CD_r * (wz_local * L_ref / (2 * V)) +
          missile_parameter.CD_de * delta_e + missile_parameter.CD_da * delta_a + missile_parameter.CD_dr * delta_r)

    # 侧向力系数 CY
    CY = (missile_parameter.CY0 + missile_parameter.CY_alpha * alpha + missile_parameter.CY_beta * beta +
          missile_parameter.CY_q * (wy_local * L_ref / (2 * V)) +
          missile_parameter.CY_p * (wx_local * L_ref / (2 * V)) +
          missile_parameter.CY_r * (wz_local * L_ref / (2 * V)) +
          missile_parameter.CY_de * delta_e + missile_parameter.CY_da * delta_a + missile_parameter.CY_dr * delta_r)

    # 俯仰力矩系数 Cm
    Cm = (missile_parameter.Cm0 + missile_parameter.Cm_alpha * alpha + missile_parameter.Cm_beta * beta +
          missile_parameter.Cm_q * (wy_local * L_ref / (2 * V)) +
          missile_parameter.Cm_alphadot * (pose_data.aoadot_radps * L_ref / (2 * V)) +
          missile_parameter.Cm_de * delta_e + missile_parameter.Cm_da * delta_a + missile_parameter.Cm_dr * delta_r)

    # 滚转力矩系数 Cl
    Cl = (missile_parameter.Cl0 + missile_parameter.Cl_alpha * alpha + missile_parameter.Cl_beta * beta +
          missile_parameter.Cl_p * (wx_local * L_ref / (2 * V)) +
          missile_parameter.Cl_r * (wz_local * L_ref / (2 * V)) +
          missile_parameter.Cl_de * delta_e + missile_parameter.Cl_da * delta_a + missile_parameter.Cl_dr * delta_r)

    # 偏航力矩系数 Cn
    Cn = (missile_parameter.Cn0 + missile_parameter.Cn_alpha * alpha + missile_parameter.Cn_beta * beta +
          missile_parameter.Cn_p * (wx_local * L_ref / (2 * V)) +
          missile_parameter.Cn_r * (wz_local * L_ref / (2 * V)) +
          missile_parameter.Cn_de * delta_e + missile_parameter.Cn_da * delta_a + missile_parameter.Cn_dr * delta_r)

    # 计算力和力矩 (单位: N, Nm)
    # 力
    D = 0.5 * rho * V_squared * S_ref * CD
    Y = 0.5 * rho * V_squared * S_ref * CY
    L = 0.5 * rho * V_squared * S_ref * CL
    
    # 力矩
    M = 0.5 * rho * V_squared * S_ref * L_ref * Cm      # 俯仰力矩
    Roll_Moment = 0.5 * rho * V_squared * S_ref * L_ref * Cl # 滚转力矩
    N = 0.5 * rho * V_squared * S_ref * L_ref * Cn      # 偏航力矩
    
    # 本体参考系下的力向量
    # 坐标系定义: 
    # x轴: 指向导弹前方
    # y轴: 指向导弹右侧
    # z轴: 指向导弹上方
    #
    # 阻力 D: 沿 -vx_local 方向 (与前进方向相反)
    # 侧向力 Y: 沿 +vy_local 方向 (右侧) 呃呃，怎么变成左手系了
    # 升力 L: 沿 +vz_local 方向 (向上) 
    # 
    # 因此，本体参考系下的力分量为:
    # Fx = -D
    # Fy = Y
    # Fz = L
    aeroforces_body = [-D, Y, L]
    power_data.F=aeroforces_body
    # 本体参考系下的力矩向量
    # 坐标系定义:
    # 绕 x轴 (Roll): 滚转力矩 L (通常右手螺旋，正向为右滚)
    # 绕 y轴 (Pitch): 俯仰力矩 M (通常右手螺旋，正向为俯)
    # 绕 z轴 (Yaw):   偏航力矩 N (通常右手螺旋，正向为右偏)
    # 
    # 现代惯导系统中，角速度通常是 p, q, r
    # p: 绕 x 轴的角速度 (滚转)
    # q: 绕 y 轴的角速度 (俯仰)
    # r: 绕 z 轴的角速度 (偏航)
    #
    # 对应的力矩系数 Cl, Cm, Cn 通常对应于绕 x, y, z 轴的力矩。
    # 因此，本体参考系下的力矩分量为:
    # Mx = Roll_Moment (对应 Cl)
    # My = M (对应 Cm)
    # Mz = N (对应 Cn)
    aeromoments_body = [Roll_Moment, M, N]
    power_data.M=aeromoments_body
    return 0

def air_power_use_step(x,body_id):
    force_in_body_frame = np.array(power_data.F, dtype=np.float64)
    torque_in_body_frame = np.array(power_data.M, dtype=np.float64)
    print(f"在 body {body_id} 的体坐标系下施加力: {force_in_body_frame}")
    print(f"在 body {body_id} 的体坐标系下施加力矩: {torque_in_body_frame}")
    # 施加力和力矩
    #mujoco.apply_body_force_torque(x.model, x.data, body_id, force_in_body_frame, torque_in_body_frame)
    mujoco.mj_applyFT(
    x.model,
    x.data ,
    force_in_body_frame  ,  # 力的大小
    torque_in_body_frame ,  # 扭矩
    x.data.xipos[body_id],  # 应用力的位置，这个参数是什么？
    body_id,  # 物体ID
    x.data.qfrc_applied,  # 应用力的数组，这个参数是什么？
    )

def debug_step(x,body_id):
    '''# 每个body的空间加速度：[angacc(3), linacc(3)]，均在世界系
    cacc = x.data.cacc.reshape(-1, 6)

    angacc_world = cacc[body_id, 0:3]
    linacc_world = cacc[body_id, 3:6]   # 这就是“包含重力”的世界线加速度 a_w

    ares = np.zeros(6, dtype=np.float64)
    mujoco.mj_objectAcceleration(x.model, x.data, mujoco.mjtObj.mjOBJ_BODY, body_id, ares,0)
    angacc_world = ares[0:3]
    linacc_world = ares[3:6]  # 含重力
    print(f"body {body_id} 的世界线加速度: {linacc_world}")
    print(f"body {body_id} 的世界角加速度: {angacc_world}")'''
    #看看哪个力不对劲

def body_acc6_at_com(m, d, body_id, local=False):
    # 确保 cacc 已刷新（你的环境里建议加这一句）
    mujoco.mj_rnePostConstraint(m, d)

    res = np.zeros(6, dtype=np.float64)
    flg_local = 1 if local else 0  # 0=world, 1=local
    mujoco.mj_objectAcceleration(m, d, mujoco.mjtObj.mjOBJ_BODY, body_id, res,1)
    angacc = res[:3].copy()
    linacc = res[3:].copy()
    return angacc, linacc

def step(x):
    body_id = x.model.body("missile").id
    pose_get(x,body_id)
    air_power_cal_step()
    debug_step(x,body_id)
    #body_acc6_at_com(x.model, x.data, body_id)
    #debug_acc(x.model, x.data, body_id)
    #air_power_use_step(x,body_id)
    mujoco.mj_step(x.model, x.data)#模拟器运行
    x.viewer.sync()#画面显示

if __name__ == "__main__":
    pass