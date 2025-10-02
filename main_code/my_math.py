import numpy as np
from scipy.spatial.transform import Rotation as R

# 你已有的工具
def normalize_quat(q):
    """ 四元数归一化（关键步骤） """
    return q / np.linalg.norm(q)

def quat_rotate_vector(q, v):
    """ 四元数旋转向量（不依赖欧拉角），q = [w, x, y, z]（MuJoCo格式） """
    w, x, y, z = q
    vx, vy, vz = v
    return np.array([
        (1 - 2*y**2 - 2*z**2)*vx + 2*(x*y - w*z)*vy + 2*(x*z + w*y)*vz,  # X
        2*(x*y + w*z)*vx + (1 - 2*x**2 - 2*z**2)*vy + 2*(y*z - w*x)*vz,  # Y
        2*(x*z - w*y)*vx + 2*(y*z + w*x)*vy + (1 - 2*x**2 - 2*y**2)*vz   # Z
    ])

def get_euler_angles(q):
    """
    MuJoCo四元数(w,x,y,z) → SciPy四元数(x,y,z,w)，然后以XYZ顺序取欧拉角（度）
    注意：仅用于调试/可视化。动力学请优先用四元数直接旋转。
    """
    scipy_q = np.roll(q, shift=-1)
    rotation = R.from_quat(scipy_q)
    euler_angles = rotation.as_euler('XYZ', degrees=True)
    return euler_angles


# 辅助：把 MuJoCo 四元数封装为 SciPy Rotation
def rot_from_mj_quat(q_wb):
    """
    q_wb: MuJoCo格式 [w,x,y,z]，表示 body->world
    返回 SciPy Rotation 对象（同样表示 body->world）
    """
    q_wb = normalize_quat(np.asarray(q_wb, dtype=np.float64))
    # SciPy需要 [x,y,z,w]
    scipy_quat = np.array([q_wb[1], q_wb[2], q_wb[3], q_wb[0]], dtype=np.float64)
    return R.from_quat(scipy_quat)


# 1) 世界 <-> 弹体
def world_to_body(vec_world, q_wb=None, R_wb=None):
    """
    世界 -> 弹体
    输入：
      - vec_world: (3,)
      - 提供 q_wb（MuJoCo四元数，body->world）或 R_wb（3x3，body->world）
    """
    v = np.asarray(vec_world, dtype=np.float64)
    if q_wb is not None:
        rot = rot_from_mj_quat(q_wb)
        # world->body 是逆旋转
        return rot.inv().apply(v)
    elif R_wb is not None:
        return (np.asarray(R_wb, dtype=np.float64).T @ v)
    else:
        raise ValueError("world_to_body: 需要提供 q_wb 或 R_wb")

def body_to_world(vec_body, q_wb=None, R_wb=None):
    """
    弹体 -> 世界
    输入：
      - vec_body: (3,)
      - 提供 q_wb（MuJoCo四元数，body->world）或 R_wb（3x3，body->world）
    """
    v = np.asarray(vec_body, dtype=np.float64)
    if q_wb is not None:
        rot = rot_from_mj_quat(q_wb)
        return rot.apply(v)
    elif R_wb is not None:
        return (np.asarray(R_wb, dtype=np.float64) @ v)
    else:
        raise ValueError("body_to_world: 需要提供 q_wb 或 R_wb")


# 2) 弹体 <-> 风轴（速度轴）
def _wind_axes_in_body(v_body):
    """
    根据弹体坐标下的速度 v_body 构造风轴到弹体的旋转矩阵（风->体），并返回风轴基向量。
    定义：
      eXw (阻力轴) = -v̂_body
      eZw 取在 x-z 平面内，尽量靠近 +z_body，且与 eXw 正交
      eYw = eZw × eXw 保持右手系
    返回：
      R_bw: (3,3)  风轴->弹体
      (eXw, eYw, eZw): 三个单位向量在弹体坐标下的表达
    """
    v_body = np.asarray(v_body, dtype=np.float64)
    vnorm = np.linalg.norm(v_body)
    if vnorm < 1e-6:
        # 速度过小，退化为单位阵（上层可在低速关气动）
        eXw = np.array([1.0, 0.0, 0.0])
        eYw = np.array([0.0, 1.0, 0.0])
        eZw = np.array([0.0, 0.0, 1.0])
        R_bw = np.eye(3)
        return R_bw, (eXw, eYw, eZw)

    eXw = -v_body / vnorm

    z_body = np.array([0.0, 0.0, 1.0])
    # 去掉 eXw 在 z 方向的投影，得到与 eXw 正交、尽量靠近 z 的方向
    eZw = z_body - np.dot(z_body, eXw) * eXw
    ezn = np.linalg.norm(eZw)
    if ezn < 1e-8:
        # 共线退化，使用 x_body 修正
        x_body = np.array([1.0, 0.0, 0.0])
        eZw = x_body - np.dot(x_body, eXw) * eXw
        eZw /= np.linalg.norm(eZw)
    else:
        eZw /= ezn

    eYw = np.cross(eZw, eXw)
    eYw /= np.linalg.norm(eYw)

    R_bw = np.column_stack((eXw, eYw, eZw))
    return R_bw, (eXw, eYw, eZw)

def body_to_wind(vec_body, v_body):
    """
    弹体 -> 风轴
    输入：
      - vec_body: (3,)
      - v_body: 弹体坐标下的速度向量（用于定义风轴）
    输出：
      - vec_wind: 风轴坐标
    """
    v = np.asarray(vec_body, dtype=np.float64)
    R_bw, _ = _wind_axes_in_body(v_body)
    R_wb = R_bw.T  # body->wind
    return R_wb @ v

def wind_to_body(vec_wind, v_body):
    """
    风轴 -> 弹体
    """
    v = np.asarray(vec_wind, dtype=np.float64)
    R_bw, _ = _wind_axes_in_body(v_body)
    return R_bw @ v


# 3) 风轴 -> 世界（合成：风->体->世）
def wind_to_world(vec_wind, q_wb=None, R_wb=None, v_body=None):
    """
    风轴 -> 世界
    需要：
      - q_wb 或 R_wb（body->world）
      - v_body（在弹体中，定义风轴）
    """
    if v_body is None:
        raise ValueError("wind_to_world: 需要 v_body 用于构造风轴")
    vbw = wind_to_body(vec_wind, v_body)

    if q_wb is not None:
        return body_to_world(vbw, q_wb=q_wb)
    elif R_wb is not None:
        return body_to_world(vbw, R_wb=R_wb)
    else:
        raise ValueError("wind_to_world: 需要 q_wb 或 R_wb")


# 4) 迎角 / 侧滑角（通过四元数把速度转到弹体）
def alpha_beta_from_world_vel(v_world, q_wb):
    """
    从世界速度 + 姿态四元数，计算弹体系的迎角alpha与侧滑beta（弧度）。
    步骤：v_body = world_to_body(v_world, q_wb) -> alpha,beta
    定义：
      alpha = atan2(Vz_body, Vx_body)
      beta  = atan2(Vy_body, Vx_body)
    """
    v_body = world_to_body(v_world, q_wb=q_wb)
    vx, vy, vz = v_body
    V = np.linalg.norm(v_body)
    if V < 1e-6:
        return 0.0, 0.0
    alpha = np.arctan2(vz, vx)
    beta  = np.arctan2(vy, vx)
    return float(alpha), float(beta)

def alpha_beta_from_body_vel(v_body):
    """
    已知弹体系速度时直接计算迎角/侧滑角（弧度）。
    """
    v_body = np.asarray(v_body, dtype=np.float64)
    vx, vy, vz = v_body
    V = np.linalg.norm(v_body)
    if V < 1e-6:
        return 0.0, 0.0
    alpha = np.arctan2(vz, vx)
    beta  = np.arctan2(vy, vx)
    return float(alpha), float(beta)