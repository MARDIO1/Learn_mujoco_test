class MissileParameter:
        def __init__(self):
            #飞行器常量
            '''# 飞行器结构参数 (Aircraft Structural Parameters)
            self.S_ref = 0.0 # 参考面积 (Reference Area) - 例如：弹翼总面积
            self.L_ref = 0.0 # 参考长度 (Reference Length) - 例如：弹体长度
            # 惯性参数 (Inertial Parameters)
            self.m = 0.0     # 飞行器质量 (Aircraft Mass)
            self.Ix = 0.0    # 绕 x 轴转动惯量 (Moment of Inertia about x-axis)
            self.Iy = 0.0    # 绕 y 轴转动惯量 (Moment of Inertia about y-axis)
            self.Iz = 0.0    # 绕 z 轴转动惯量 (Moment of Inertia about z-axis)
            self.Ixy = 0.0   # 惯量积 (Product of Inertia)
            self.Iyz = 0.0   # 惯量积
            self.Izx = 0.0   # 惯量积
            #或者用一个惯性张量矩阵表示
            
            # --- 空气动力系数 (Aerodynamic Coefficients) ---
            # 零值系数 (Zero-Value Coefficients)
            self.CL0 = 0.0; self.CD0 = 0.0; self.CY0 = 0.0
            self.Cm0 = 0.0; self.Cl0 = 0.0; self.Cn0 = 0.0
            # 迎角相关系数 (Angle of Attack Coefficients)
            self.CL_alpha = 0.0; self.CD_alpha = 0.0; self.CY_alpha = 0.0
            self.Cm_alpha = 0.0; self.Cl_alpha = 0.0; self.Cn_alpha = 0.0
            # 侧滑角相关系数 (Sideslip Angle Coefficients)
            self.CL_beta = 0.0; self.CD_beta = 0.0; self.CY_beta = 0.0
            self.Cm_beta = 0.0; self.Cl_beta = 0.0; self.Cn_beta = 0.0
            # 角速度相关系数 (Angular Velocity Coefficients)
            # 俯仰角速度 q
            self.CL_q = 0.0; self.CD_q = 0.0; self.CY_q = 0.0
            self.Cm_q = 0.0; self.Cl_q = 0.0; self.Cn_q = 0.0
            # 滚转角速度 p
            self.CL_p = 0.0; self.CD_p = 0.0; self.CY_p = 0.0
            self.Cm_p = 0.0; self.Cl_p = 0.0; self.Cn_p = 0.0
            # 偏航角速度 r
            self.CL_r = 0.0; self.CD_r = 0.0; self.CY_r = 0.0
            self.Cm_r = 0.0; self.Cl_r = 0.0; self.Cn_r = 0.0
            self.Cm_alphadot = 0.0# 迎角变化率相关系数 (Rate of Change of Angle of Attack Coefficients)
            '''
            # 飞行器结构参数 (Aircraft Structural Parameters)
            self.S_ref = 0.005 # 参考面积 (Reference Area) - 例如：弹翼总面积
            self.L_ref = 0.2 # 参考长度 (Reference Length) - 例如：弹体长度
            # 惯性参数 (Inertial Parameters)
            self.m = 1.0     # 飞行器质量 (Aircraft Mass)
            self.Ix = 0.0042    # 绕 x 轴转动惯量 (Moment of Inertia about x-axis)
            self.Iy = 0.0142    # 绕 y 轴转动惯量 (Moment of Inertia about y-axis)
            self.Iz = 0.0158    # 绕 z 轴转动惯量 (Moment of Inertia about z-axis)
            self.Ixy = 0.0   # 惯量积 (Product of Inertia)
            self.Iyz = 0.0   # 惯量积
            self.Izx = 0.0   # 惯量积
            #或者用一个惯性张量矩阵表示
            
            # --- 空气动力系数 (Aerodynamic Coefficients) ---
            # 零值系数 (Zero-Value Coefficients)
            self.CL0 = 0.0; self.CD0 = 0.1; self.CY0 = 0.0
            self.Cm0 = 0.0; self.Cl0 = 0.0; self.Cn0 = 0.0
            # 迎角相关系数 (Angle of Attack Coefficients)
            self.CL_alpha = 3.0; self.CD_alpha = 0.5; self.CY_alpha = 0.2
            self.Cm_alpha = -1.0; self.Cl_alpha = 0.0; self.Cn_alpha = 0.0
            # 侧滑角相关系数 (Sideslip Angle Coefficients)
            self.CL_beta = -0.1; self.CD_beta = 0.2; self.CY_beta = 2.0
            self.Cm_beta = 0.0; self.Cl_beta = 0.0; self.Cn_beta = -1.5
            # 角速度相关系数 (Angular Velocity Coefficients)
            # 俯仰角速度 q
            self.CL_q = 0.0; self.CD_q = 0.0; self.CY_q = 0.0
            self.Cm_q = -5.0; self.Cl_q = 0.0; self.Cn_q = 0.0
            # 滚转角速度 p
            self.CL_p = 0.0; self.CD_p = 0.0; self.CY_p = 0.0
            self.Cm_p = 0.0; self.Cl_p = -3.0; self.Cn_p = 0.0
            # 偏航角速度 r
            self.CL_r = 0.0; self.CD_r = 0.0; self.CY_r = 0.0
            self.Cm_r = 0.0; self.Cl_r = 0.2; self.Cn_r = -4.0
            self.Cm_alphadot = 0.0 # 迎角变化率相关系数 (Rate of Change of Angle of Attack Coefficients)
            #对应舵面如果没有就要设为0
            # 舵面偏转相关系数 (Control Surface Deflection Coefficients)
            # 升降舵/配平舵 de
            self.CL_de = 0.0; self.CD_de = 0.0; self.CY_de = 0.0
            self.Cm_de = 0.0; self.Cl_de = 0.0; self.Cn_de = 0.0
            # 副翼/副翼舵 da
            self.CL_da = 0.0; self.CD_da = 0.0; self.CY_da = 0.0
            self.Cm_da = 0.0; self.Cl_da = 0.0; self.Cn_da = 0.0
            # 方向舵/尾舵 dr
            self.CL_dr = 0.0; self.CD_dr = 0.0; self.CY_dr = 0.0
            self.Cm_dr = 0.0; self.Cl_dr = 0.0; self.Cn_dr = 0.0

#位姿参数
class Pose_data:
    def __init__(self):
        #通常值
        self.q_wb=[1.0,0.0,0.0,0.0]#获取的四元数
        self.yaw_pitch_roll_rad=[0.0,0.0,0.0]
        self.yaw_pitch_roll_d=[0.0,0.0,0.0]

        self.v_wind=0.0
        self.v_global_mps=[0.0,0.0,0.0]
        self.v_local_mps=[0.0,0.0,0.0]
        self.a_local_mps2=[0.0,0.0,0.0]

        self.w_wind=[0.0,0.0,0.0]
        self.w_local_radps=[0.0,0.0,0.0]
        self.w_global_radps=[0.0,0.0,0.0]
        self.w_wind=[0.0,0.0,0.0]
        self.b_local_radps2=[0.0,0.0,0.0]
        #特殊值
        self.aoa_rad=0
        self.soa_rad=0
        self.aoa_degree=0
        self.soa_degree=0

        self.last_aoa_rad=0
        self.last_soa_rad=0
        self.aoadot_radps=0#差点忘记他了
        self.soadot_radps=0
        #可直接控制值
        self.duo_y_rad=0
        self.duo_z_rad=0

    def print_debug_info(self):
        #print(f"yaw_pitch_roll_rad: {self.yaw_pitch_roll_rad}")
        print(f"yaw_pitch_roll_d: {self.yaw_pitch_roll_d}")
        print(f"v_global_mps: {self.v_global_mps}")
        #print(f"v_local_mps: {self.v_local_mps}")
        print(f"a_local_mps2: {self.a_local_mps2}")
        #print(f"w_local_radps: {self.w_local_radps}")
        #print(f"aoa_rad: {self.aoa_rad}")
        #print(f"soa_rad: {self.soa_rad}")
        #print(f"aoa_degree: {self.aoa_degree}")
        #print(f"soa_degree: {self.soa_degree}")
        #print(f"last_aoa_rad: {self.last_aoa_rad}")
        #print(f"last_soa_rad: {self.last_soa_rad}")
        #print(f"aoadot_radps: {self.aoadot_radps}")
        #print(f"soadot_radps: {self.soadot_radps}")
class Power_data:
     def __init__(self):
        # 因此，本体参考系下的力分量为:
        # Fx = -D
        # Fy = Y
        # Fz = L
        self.F_wind=[0.0,0.0,0.0]
        self.F_local=[0.0,0.0,0.0]
        self.F_global=[0.0,0.0,0.0]
        # 坐标系定义:
        # 绕 x轴 (Roll): 滚转力矩 L (通常右手螺旋，正向为右滚)
        # 绕 y轴 (Pitch): 俯仰力矩 M (通常右手螺旋，正向为俯)
        # 绕 z轴 (Yaw):   偏航力矩 N (通常右手螺旋，正向为右偏)
        self.M_local=[0.0,0.0,0.0]
        self.M_global=[0.0,0.0,0.0]

if __name__ == "__main__":
    pass