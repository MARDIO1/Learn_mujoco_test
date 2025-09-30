import numpy as np
from scipy.spatial.transform import Rotation as R
def normalize_quat(q):
    """ 四元数归一化（关键步骤） """
    return q / np.linalg.norm(q)

def quat_rotate_vector(q, v):
    """ 四元数旋转向量（不依赖欧拉角） """
    w, x, y, z = q
    vx, vy, vz = v
    return np.array([
        (1 - 2*y**2 - 2*z**2)*vx + 2*(x*y - w*z)*vy + 2*(x*z + w*y)*vz,  # X分量
        2*(x*y + w*z)*vx + (1 - 2*x**2 - 2*z**2)*vy + 2*(y*z - w*x)*vz,  # Y分量
        2*(x*z - w*y)*vx + 2*(y*z + w*x)*vy + (1 - 2*x**2 - 2*y**2)*vz   # Z分量
    ])

def get_euler_angles(q):
    # MuJoCo四元数(w,x,y,z) → SciPy四元数(x,y,z,w)
    scipy_q = np.roll(q, shift=-1)
    rotation = R.from_quat(scipy_q)
    
    # 使用XYZ顺序（Roll-Pitch-Yaw）
    euler_angles = rotation.as_euler('XYZ', degrees=True)
    return euler_angles