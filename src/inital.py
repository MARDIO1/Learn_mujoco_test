import mujoco
import mujoco.viewer
import os
import rerun as rr
# 各项参数用来快速调整
xml_file_path = r"..\\models\\all.xml"
urdf_file_path= "..\\models\\try2.SLDASM\\urdf\\try2.SLDASM.urdf"

# 乱七八糟的辅助函数
# 第一个就难绷完了，为什么新版本还把自由度数量删了，还要自己用type推断？而且为什么只有这么几种自由度？就没有2,4吗
def get_dof_count(model, joint_id) -> int:
    """根据关节类型返回其在qvel中的自由度数量。"""
    joint_type = model.jnt_type[joint_id]
    if joint_type == mujoco.mjtJoint.mjJNT_FREE:
        return 6
    elif joint_type == mujoco.mjtJoint.mjJNT_BALL:
        return 3
    elif (
        joint_type == mujoco.mjtJoint.mjJNT_HINGE
        or joint_type == mujoco.mjtJoint.mjJNT_SLIDER
    ):
        return 1
    else:
        print(
            f"Warning: Unknown joint type {joint_type} for joint ID {joint_id}. Returning 0 DOFs."
        )
        return 0


# 封装了一个超级重要的类
class MujocoHandler:
    def __init__(self, path):
        # xml文件参数#一些大接口参数
        self.mjcf_file_path = os.path.join(os.path.dirname(__file__), path)
        self.model = mujoco.MjModel.from_xml_path(self.mjcf_file_path)
        # self.scn = self.scene = mujoco.MjvScene(self.model, maxgeom=10000)
        self.data = mujoco.MjData(self.model)
        self.viewer = None  # 修改为 self.viewer = None
        # 一些自定义小接口参数
        self.body_id = {}
        self.joint_id = {}
        self.motor_id = {}

    # 显示画面大函数
    def launch_viewer(self):
        """启动 viewer 窗口"""
        if self.viewer is None:
            self.viewer = mujoco.viewer.launch_passive(self.model, self.data)
            mujoco.mjv_defaultFreeCamera(self.model, self.viewer.cam)

            # 手动设置自由相机参数
            self.viewer.cam.type = mujoco.mjtCamera.mjCAMERA_FREE
            self.viewer.cam.lookat[:] = [
                13,
                0,
                1.0,
            ]  # 场景中点，大概看中间（Z 调成 1 m 高度）
            self.viewer.cam.distance = 40  # 缩放，使目标和起点都进画面
            self.viewer.cam.azimuth = 90  # 90° 表示沿 Y 轴负方向→X 正方向看
            self.viewer.cam.elevation = 0  # 水平
            self.viewer.cam.orthographic = 1  # 正交投影

    # id对应名字,构造函数的一部分
    def get_all_body_ids(self):
        print("总共有", self.model.nbody, "个body")
        for i in range(self.model.nbody):  # model.nbody 是模型中 body 的总数
            body_name = self.model.body(i).name  # 获取 body 的名称，注意新版本
            if body_name:  # 确保 body 有名称 (有些 body 可能没有名称)
                self.body_id[body_name] = i  # 将名称和 ID 添加到字典中
            print(i, " ", body_name)

    # 关节对应索引，自由度二维数组，构造函数的一部分
    def get_all_joint_ids(self):
        print("总共有", self.model.njnt, "个关节")
        for i in range(self.model.njnt):  # model.njnt 是模型中 joint 的总数
            joint_name = mujoco.mj_id2name(
                self.model, mujoco.mjtObj.mjOBJ_JOINT, i
            )  # 获取 joint 的名称，注意新版本
            if joint_name:
                temp_qpos_adr = self.model.jnt_qposadr[i]
                temp_qvel_adr = self.model.jnt_dofadr[i]
                temp_dof_num = get_dof_count(self.model, i)
                self.joint_id[joint_name] = {
                    "id": i,
                    "name": joint_name,
                    "qpos_adr": temp_qpos_adr,
                    "qvel_adr": temp_qvel_adr,
                    "dof_num": temp_dof_num,
                }
            print(self.joint_id[joint_name])

        # id对应名字,构造函数的一部分

    def get_all_motors_ids(self):
        print("总共有", self.model.nu, "个motor")
        for i in range(self.model.nu):  # model.nbody 是模型中 body 的总数
            motor_name = self.model.id2name(
                mujoco.mjtObj.mjOBJ_MOTOR, i
            )  # 获取 body 的名称，注意新版本
            if motor_name:  # 确保 body 有名称 (有些 body 可能没有名称)
                self.motor_id[motor_name] = i  # 将名称和 ID 添加到字
            print(i, " ", motor_name)

    def render_overlay(model, data, scn):
        return 0

def rr_inital():
    #高级示波器
    rr.init("rerun_example_scalar_row_updates", spawn=True)
    rr.log("AOA", rr.SeriesLines(colors=[125, 0, 0], names="sin(0.01t)", widths=4), static=True)
    rr.log("SOA", rr.SeriesLines(colors=[0, 125, 125], names="cos(0.01t)", widths=4), static=True)

def inital() -> MujocoHandler:
    #urdf文件

    # 加载文件，初始化
    print("母鸡卡 启动！")
    rr_inital()
    mujoco_handler = MujocoHandler(xml_file_path)
    mujoco_handler.get_all_body_ids()
    mujoco_handler.get_all_joint_ids()
    mujoco_handler.get_all_motors_ids()
    print("模型和类初始化 胜利！")
    mujoco_handler.launch_viewer()
    print("free camera 初始化 胜利！")

    return mujoco_handler


if __name__ == "__main__":
    pass
