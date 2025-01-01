import sys
import open3d as o3d
import numpy as np
import math
import multiprocessing
import queue
import time
from PyQt5.QtWidgets import QApplication, QWidget, QVBoxLayout, QSlider, QSpinBox, QLabel, QHBoxLayout, QTextEdit
from PyQt5.QtCore import Qt

# -------------------------------------
# 定义 Transform 类
# -------------------------------------
class Transform:
    """
    提供各种变换矩阵的生成函数,包括平移和旋转。
    """
    @staticmethod
    def multiplyT(arr):
        """
        将一系列变换矩阵按顺序相乘,返回最终的累积变换矩阵。

        参数:
            arr (list of np.array): 要相乘的变换矩阵列表。

        返回:
            np.array: 累积后的4x4变换矩阵。
        """
        T = np.eye(4)  # 初始化为单位矩阵
        for transform in arr:
            T = np.dot(T, transform)  # 按顺序相乘
        return T

    @staticmethod
    def xmoveT(x):
        """
        创建沿X轴平移的变换矩阵。

        参数:
            x (float): 沿X轴平移的距离。

        返回:
            np.array: 4x4平移矩阵。
        """
        return np.array([[1, 0, 0, x],
                         [0, 1, 0, 0],
                         [0, 0, 1, 0],
                         [0, 0, 0, 1]])

    @staticmethod
    def ymoveT(y):
        """
        创建沿Y轴平移的变换矩阵。

        参数:
            y (float): 沿Y轴平移的距离。

        返回:
            np.array: 4x4平移矩阵。
        """
        return np.array([[1, 0, 0, 0],
                         [0, 1, 0, y],
                         [0, 0, 1, 0],
                         [0, 0, 0, 1]])

    @staticmethod
    def zmoveT(z):
        """
        创建沿Z轴平移的变换矩阵。

        参数:
            z (float): 沿Z轴平移的距离。

        返回:
            np.array: 4x4平移矩阵。
        """
        return np.array([[1, 0, 0, 0],
                         [0, 1, 0, 0],
                         [0, 0, 1, z],
                         [0, 0, 0, 1]])

    @staticmethod
    def xrotateT(angle):
        """
        创建绕X轴旋转的变换矩阵。

        参数:
            angle (float): 旋转角度（弧度）。

        返回:
            np.array: 4x4旋转矩阵。
        """
        return np.array([[1, 0, 0, 0],
                         [0, np.cos(angle), -np.sin(angle), 0],
                         [0, np.sin(angle), np.cos(angle), 0],
                         [0, 0, 0, 1]])

    @staticmethod
    def yrotateT(angle):
        """
        创建绕Y轴旋转的变换矩阵。

        参数:
            angle (float): 旋转角度（弧度）。

        返回:
            np.array: 4x4旋转矩阵。
        """
        return np.array([[np.cos(angle), 0, np.sin(angle), 0],
                         [0, 1, 0, 0],
                         [-np.sin(angle), 0, np.cos(angle), 0],
                         [0, 0, 0, 1]])

    @staticmethod
    def zrotateT(angle):
        """
        创建绕Z轴旋转的变换矩阵。

        参数:
            angle (float): 旋转角度（弧度）。

        返回:
            np.array: 4x4旋转矩阵。
        """
        return np.array([[np.cos(angle), -np.sin(angle), 0, 0],
                         [np.sin(angle), np.cos(angle), 0, 0],
                         [0, 0, 1, 0],
                         [0, 0, 0, 1]])

# -------------------------------------
# 定义复杂变换函数
# -------------------------------------
def base_raw_rotateT(axis, angle_input, rotate_center, rotate_raw, transform_obj):
    """
    根据指定的轴和角度,生成旋转矩阵和整体变换矩阵。

    参数:
        axis (int): 关节轴编号（从2开始）。
        angle_input (float): 旋转角度（弧度）。
        rotate_center (list of list): 每个关节的旋转中心坐标。
        rotate_raw (list of list): 每个关节的旋转法向量。
        transform_obj (Transform): 变换对象,用于生成变换矩阵。

    返回:
        tuple: (旋转矩阵 Tr, 完整变换矩阵 T)
    """
    center = rotate_center[axis-2]  # 获取当前关节的旋转中心
    raw = rotate_raw[axis-2]        # 获取当前关节的旋转法向量

    # 计算旋转矩阵 Tr（局部旋转）
    if raw == [1, 0, 0]:  # X轴
        Tr = transform_obj.xrotateT(angle_input)
    elif raw == [0, 1, 0]:  # Y轴
        Tr = transform_obj.yrotateT(angle_input)
    elif raw == [0, 0, 1]:  # Z轴
        Tr = transform_obj.zrotateT(angle_input)
    elif raw == [0, -1, 0]:  # -Y轴
        Tr = transform_obj.yrotateT(angle_input)
    elif raw == [0, 0, -1]:  # -Z轴
        Tr = transform_obj.zrotateT(angle_input)
    else:
        # 对于非标准轴的旋转,需更复杂的处理（暂时返回单位矩阵）
        Tr = np.eye(4)

    # 生成完整变换矩阵 T：先平移到旋转中心,旋转,再平移回来
    T = transform_obj.multiplyT([
        transform_obj.xmoveT(center[0]),
        transform_obj.ymoveT(center[1]),
        transform_obj.zmoveT(center[2]),
        Tr,
        transform_obj.xmoveT(-center[0]),
        transform_obj.ymoveT(-center[1]),
        transform_obj.zmoveT(-center[2])
    ])

    return Tr, T

# -------------------------------------
# 定义原始旋转函数
# -------------------------------------
def raw_rotate(axis, angle_input, mesh_dict, rotate_center, rotate_raw, transform_obj):
    """
    根据关节轴和输入角度,计算变换矩阵并应用到相应的Mesh上。

    参数:
        axis (int): 关节轴编号（从2开始）。
        angle_input (float): 旋转角度（弧度）。
        mesh_dict (dict): 存储所有Mesh的字典。
        rotate_center (list of list): 每个关节的旋转中心坐标。
        rotate_raw (list of list): 每个关节的旋转法向量。
        transform_obj (Transform): 变换对象,用于生成变换矩阵。

    返回:
        None
    """
    Tr, T = base_raw_rotateT(axis, angle_input, rotate_center, rotate_raw, transform_obj)
    # 检查轴编号是否在有效范围内
    if axis in range(2, 8):
        # 遍历从当前轴到第7个Mesh,依次应用变换
        for i in range(axis, 8):
            mesh_name = f'mesh{i}'
            mesh_dict[mesh_name].transform(T)  # 应用整体变换
            # 更新旋转中心位置
            rotate_center[i-2] = np.dot(T, np.array(rotate_center[i-2]))
            # 更新旋转法向量方向
            rotate_raw[i-2] = np.dot(Tr[:3, :3], np.array(rotate_raw[i-2]))
    else:
        # 如果轴编号不在预期范围内,忽略变换
        pass
    return

# -------------------------------------
# 手动复制 TriangleMesh 对象
# -------------------------------------
def manual_copy_mesh(mesh):
    """
    手动复制一个 TriangleMesh 对象,创建一个新的实例并复制其属性。

    参数:
        mesh (o3d.geometry.TriangleMesh): 要复制的Mesh对象。

    返回:
        o3d.geometry.TriangleMesh: 复制后的新Mesh对象。
    """
    new_mesh = o3d.geometry.TriangleMesh()
    # 复制顶点
    new_mesh.vertices = o3d.utility.Vector3dVector(np.asarray(mesh.vertices))
    # 复制三角形面
    new_mesh.triangles = o3d.utility.Vector3iVector(np.asarray(mesh.triangles))
    
    # 如果有顶点法向量,则复制
    if mesh.has_vertex_normals():
        new_mesh.vertex_normals = o3d.utility.Vector3dVector(np.asarray(mesh.vertex_normals))
    
    # 如果有顶点颜色,则复制
    if mesh.has_vertex_colors():
        new_mesh.vertex_colors = o3d.utility.Vector3dVector(np.asarray(mesh.vertex_colors))
    
    return new_mesh

# -------------------------------------
# RobotVisualizer 类
# -------------------------------------
class RobotVisualizer(multiprocessing.Process):
    """
    负责加载、管理和可视化机器人各个部件的Mesh对象,并根据指令更新其姿态。

    属性:
        command_queue (multiprocessing.Queue): 用于接收来自主进程的指令队列。
        mesh_dict (dict): 存储所有Mesh对象的字典。
        original_mesh_dict (dict): 存储Mesh对象原始状态的字典,用于重置。
        base_mesh (dict): 存储基础Mesh用于变换重置。
        rotate_center (list of list): 每个关节的旋转中心坐标。
        rotate_raw (list of list): 每个关节的旋转法向量。
        joint_angles (list of float): 当前六个关节的角度值（度）。
        running (bool): 控制可视化循环是否继续运行。
    """
    def __init__(self, command_queue):
        super(RobotVisualizer, self).__init__()
        self.command_queue = command_queue  # 初始化命令队列
        self.mesh_dict = {}  # 存储当前Mesh对象
        self.original_mesh_dict = {}  # 存储原始Mesh对象
        self.base_mesh = {}  # 存储基础Mesh用于重置
        self.rotate_center = []  # 存储每个关节的旋转中心
        self.rotate_raw = []  # 存储每个关节的旋转法向量
        self.joint_angles = [0.0 for _ in range(6)]  # 初始化六个关节角度为0度
        self.running = True  # 控制可视化循环
        self.transform_obj = Transform()  # 创建Transform对象
        self.log_queue = multiprocessing.Queue()  # 用于日志记录

    def run(self):
        """
        进程启动后执行的方法,负责加载Mesh、初始化可视化窗口,并循环监听指令进行更新。
        """
        try:
            # 加载Mesh和定义旋转中心及法向量
            self.load_meshes()

            # 定义初始坐标系变换矩阵 T0（将Y轴和Z轴交换,并翻转Y轴）
            T0 = np.array([[1, 0, 0, 0],
                          [0, 0, -1, 0],
                          [0, 1, 0, 0],
                          [0, 0, 0, 1]])

            # 应用初始变换 T0 到所有Mesh
            for mesh_name in ['mesh1', 'mesh2', 'mesh3', 'mesh4', 'mesh5', 'mesh6', 'mesh7']:
                self.mesh_dict[mesh_name].transform(T0)
                # 使用 copy() 进行深拷贝,如果不可用则手动复制
                if hasattr(self.mesh_dict[mesh_name], 'copy'):
                    self.original_mesh_dict[mesh_name] = self.mesh_dict[mesh_name].copy()
                else:
                    self.original_mesh_dict[mesh_name] = manual_copy_mesh(self.mesh_dict[mesh_name])

            # 保存基础Mesh用于后续的重置操作
            self.base_mesh = {}
            for mesh_name in ['mesh1', 'mesh2', 'mesh3', 'mesh4', 'mesh5', 'mesh6', 'mesh7']:
                if hasattr(self.original_mesh_dict[mesh_name], 'copy'):
                    self.base_mesh[mesh_name] = self.original_mesh_dict[mesh_name].copy()
                else:
                    self.base_mesh[mesh_name] = manual_copy_mesh(self.original_mesh_dict[mesh_name])

            # 初始化Open3D可视化窗口
            self.vis = o3d.visualization.Visualizer()
            self.vis.create_window(window_name='🤖 UR3机器人仿真', width=800, height=600)
            # 添加一个坐标系作为参考
            coord1 = o3d.geometry.TriangleMesh.create_coordinate_frame(size=100.0)
            self.vis.add_geometry(coord1)
            # 将所有Mesh添加到可视化窗口中
            for i in range(1, 8):
                self.vis.add_geometry(self.mesh_dict[f'mesh{i}'])

            self.log("✨ 初始化完成。准备接受指令,启动运动学仿真！")

            # 可视化循环,持续监听并处理指令
            while self.running:
                try:
                    command = self.command_queue.get_nowait()  # 非阻塞获取指令
                    if command['type'] == 'set_angle':
                        axis = command['axis']  # 获取关节轴编号
                        angle = command['angle']  # 获取关节角度（度）
                        angle_rad = math.radians(angle)  # 将角度转换为弧度
                        if 1 <= axis <= 6:
                            self.joint_angles[axis-1] = angle  # 更新关节角度列表
                            self.update_robot()  # 根据新的角度更新机器人模型
                            # 打印格式化信息,反馈更新状态
                            self.log(f"✨ 关节 {axis} 已成功设置为 {angle} 度。运动学仿真已更新！")
                        else:
                            self.log(f"⚠️ 无效的关节编号：{axis}")
                    elif command['type'] == 'exit':
                        # 收到退出命令,结束可视化循环
                        self.log("🛑 收到退出命令,正在关闭可视化窗口。")
                        self.running = False
                except queue.Empty:
                    # 如果队列为空,继续循环
                    pass

                # 处理Open3D的事件和渲染
                self.vis.poll_events()
                self.vis.update_renderer()
                time.sleep(0.01)  # 每10毫秒更新一次

            # 退出循环后,销毁可视化窗口
            self.vis.destroy_window()
            self.log("👋 可视化窗口已关闭。运动学仿真结束。")
        except Exception as e:
            self.log(f"❌ 发生未捕捉的异常: {e}")
            self.running = False

    def update_robot(self):
        """
        根据当前关节角度,重新计算并应用所有关节的变换,更新机器人模型。
        """
        try:
            # 重置所有Mesh到基础状态,除了底座mesh1
            for mesh_name in ['mesh2', 'mesh3', 'mesh4', 'mesh5', 'mesh6', 'mesh7']:
                # 恢复顶点、三角形和法向量
                mesh = self.mesh_dict[mesh_name]
                base_mesh = self.base_mesh[mesh_name]
                mesh.vertices = o3d.utility.Vector3dVector(np.asarray(base_mesh.vertices))
                mesh.triangles = o3d.utility.Vector3iVector(np.asarray(base_mesh.triangles))
                mesh.vertex_normals = o3d.utility.Vector3dVector(np.asarray(base_mesh.vertex_normals))
                # 恢复顶点颜色或统一颜色
                if base_mesh.has_vertex_colors():
                    mesh.vertex_colors = o3d.utility.Vector3dVector(np.asarray(base_mesh.vertex_colors))
                else:
                    mesh.paint_uniform_color([1, 1, 1])  # 默认白色

            # 初始化累积变换矩阵
            cumulative_T = np.eye(4)

            # 按关节顺序应用变换
            for i in range(6):
                angle_rad = math.radians(self.joint_angles[i])  # 当前关节角度（弧度）
                axis = i + 2  # 关节轴编号从2开始（mesh2对应关节1）
                Tr, T = base_raw_rotateT(axis, angle_rad, self.rotate_center, self.rotate_raw, self.transform_obj)  # 获取旋转和变换矩阵
                cumulative_T = np.dot(cumulative_T, T)  # 更新累积变换矩阵

                # 对应的Mesh名称
                mesh_name = f'mesh{i+2}'
                mesh = self.mesh_dict[mesh_name]

                # 应用累积变换到对应的Mesh
                mesh.transform(cumulative_T)

            # 发送姿态信息到主进程（暂未实现）
            self.log_queue.put("✅ 机器人模型已更新。")

            # 更新可视化窗口中的几何体
            for i in range(2, 8):
                self.vis.update_geometry(self.mesh_dict[f'mesh{i}'])
        except Exception as e:
            self.log(f"❌ 更新机器人模型时出错: {e}")

    def load_meshes(self):
        """
        加载所有Robot的Mesh文件,并初始化颜色和计算法向量。
        """
        try:
            # 依次加载各个部件的STL文件
            self.mesh_dict['mesh1'] = o3d.io.read_triangle_mesh(
                r"D:\360MoveData\Users\12978\Desktop\Open3D+python大作业\Models\UR3 - 31220-STEP-1.STL")
            self.mesh_dict['mesh2'] = o3d.io.read_triangle_mesh(
                r"D:\360MoveData\Users\12978\Desktop\Open3D+python大作业\Models\UR3 - 31602-STEP-1.STL")
            self.mesh_dict['mesh3'] = o3d.io.read_triangle_mesh(
                r"D:\360MoveData\Users\12978\Desktop\Open3D+python大作业\Models\UR3 - dabi-1 31126-STEP-1.STL")
            # 将多个Mesh合并到mesh3中
            self.mesh_dict['mesh3'] += o3d.io.read_triangle_mesh(
                r"D:\360MoveData\Users\12978\Desktop\Open3D+python大作业\Models\UR3 - dabi-1 31601-STEP-1.STL")
            self.mesh_dict['mesh3'] += o3d.io.read_triangle_mesh(
                r"D:\360MoveData\Users\12978\Desktop\Open3D+python大作业\Models\UR3 - dabi-1 31602-STEP-1.STL")
            self.mesh_dict['mesh3'] += o3d.io.read_triangle_mesh(
                r"D:\360MoveData\Users\12978\Desktop\Open3D+python大作业\Models\UR3 - dabi-1 34111-STEP-1.STL")
            self.mesh_dict['mesh4'] = o3d.io.read_triangle_mesh(
                r"D:\360MoveData\Users\12978\Desktop\Open3D+python大作业\Models\UR3 - xiaobi-1 31116-STEP-1.STL")
            self.mesh_dict['mesh4'] += o3d.io.read_triangle_mesh(
                r"D:\360MoveData\Users\12978\Desktop\Open3D+python大作业\Models\UR3 - xiaobi-1 31690-STEP-1.STL")
            self.mesh_dict['mesh5'] = o3d.io.read_triangle_mesh(
                r"D:\360MoveData\Users\12978\Desktop\Open3D+python大作业\Models\UR3 - 装配体7-1 31600-STEP-1.STL")
            self.mesh_dict['mesh5'] += o3d.io.read_triangle_mesh(
                r"D:\360MoveData\Users\12978\Desktop\Open3D+python大作业\Models\UR3 - 装配体7-1 34011-STEP-1.STL")
            self.mesh_dict['mesh6'] = o3d.io.read_triangle_mesh(
                r"D:\360MoveData\Users\12978\Desktop\Open3D+python大作业\Models\UR3 - 装配体8-1 31600-STEP-1.STL")
            self.mesh_dict['mesh6'] += o3d.io.read_triangle_mesh(
                r"D:\360MoveData\Users\12978\Desktop\Open3D+python大作业\Models\UR3 - 装配体8-1 34011-STEP-1.STL")
            self.mesh_dict['mesh7'] = o3d.io.read_triangle_mesh(
                r"D:\360MoveData\Users\12978\Desktop\Open3D+python大作业\Models\UR3 - 装配体9-1 31000-STEP-1.STL")
            self.mesh_dict['mesh7'] += o3d.io.read_triangle_mesh(
                r"D:\360MoveData\Users\12978\Desktop\Open3D+python大作业\Models\UR3 - 装配体9-1 31600-STEP-1.STL")
            self.mesh_dict['mesh7'] += o3d.io.read_triangle_mesh(
                r"D:\360MoveData\Users\12978\Desktop\Open3D+python大作业\Models\UR3 - 装配体9-1 34011-STEP-1.STL")
            self.log("✅ 所有STL文件加载成功。")
        except Exception as e:
            # 如果加载过程中出现错误,打印错误信息并退出程序
            self.log(f"❌ 加载STL文件时出错: {e}")
            sys.exit(1)
        
        # 为每个Mesh指定颜色,以便在可视化中区分不同部分
        self.mesh_dict['mesh1'].paint_uniform_color([1, 0, 0])    # 红色
        self.mesh_dict['mesh2'].paint_uniform_color([0, 1, 0])    # 绿色
        self.mesh_dict['mesh3'].paint_uniform_color([0, 0, 1])    # 蓝色
        self.mesh_dict['mesh4'].paint_uniform_color([1, 1, 0])    # 黄色
        self.mesh_dict['mesh5'].paint_uniform_color([1, 0, 1])    # 品红色
        self.mesh_dict['mesh6'].paint_uniform_color([0, 1, 1])    # 青色
        self.mesh_dict['mesh7'].paint_uniform_color([1, 0.5, 0])  # 橙色

        # 计算每个Mesh的顶点、面和法向量数量,并打印
        for i, mesh in enumerate([self.mesh_dict['mesh1'], self.mesh_dict['mesh2'], self.mesh_dict['mesh3'],
                                  self.mesh_dict['mesh4'], self.mesh_dict['mesh5'], self.mesh_dict['mesh6'],
                                  self.mesh_dict['mesh7']], start=1):
            mesh.compute_vertex_normals()  # 计算顶点法向量,用于更好的渲染效果
            self.log(f"🔹 Mesh{i} - 顶点数：{len(mesh.vertices)}, 面数：{len(mesh.triangles)}, 法向量数：{len(mesh.vertex_normals)}")

        # 定义每个关节的旋转中心和旋转法向量（仅3个元素）
        self.rotate_center = [
            [64.63999939, 86.97676086, 194.93330383],    # Joint1: mesh2
            [64.63999939, 152.75356814, 140.93330383],   # Joint2: mesh3
            [64.63999939, 396.47674561, 117.33331299],   # Joint3: mesh4
            [64.63999939, 574.2767334,  167.93330383],    # Joint4: mesh5
            [64.63999939, 609.47674561, 125.33331299],    # Joint5: mesh6
            [64.63999939, 652.07672119,  84.53330994]     # Joint6: mesh7
        ]
        
        self.rotate_raw = [
            [0, 1, 0],   # Joint1: Y轴
            [0, 0, -1],  # Joint2: -Z轴
            [0, 0, 1],   # Joint3: Z轴
            [0, 1, 0],   # Joint4: Y轴
            [0, 0, -1],  # Joint5: -Z轴
            [0, 1, 0]    # Joint6: Y轴
        ]

    def log(self, message):
        """
        将日志消息发送到日志队列,用于在主进程中显示。

        参数:
            message (str): 要记录的日志消息。
        """
        self.log_queue.put(message)

    def get_logs(self):
        """
        从日志队列中获取所有日志消息。

        返回:
            list of str: 日志消息列表。
        """
        logs = []
        while not self.log_queue.empty():
            logs.append(self.log_queue.get())
        return logs

# -------------------------------------
# MainWindow 类
# -------------------------------------
class MainWindow(QWidget):
    """
    创建一个PyQt5的主窗口,包含六个关节的控制滑块和输入框,并显示状态日志和当前姿态信息。

    属性:
        command_queue (multiprocessing.Queue): 用于发送指令给Visualizer的队列。
        log_queue (multiprocessing.Queue): 用于接收Visualizer的日志消息。
        labels (list of QLabel): 存储关节标签的列表。
        sliders (list of QSlider): 存储滑块控件的列表。
        spin_boxes (list of QSpinBox): 存储输入框控件的列表。
        angle_labels (list of QLabel): 存储显示角度值的标签列表。
        log_text (QTextEdit): 显示日志消息的文本编辑器。
        pose_label (QLabel): 显示当前姿态信息的标签。
    """
    def __init__(self, command_queue, log_queue):
        super().__init__()
        self.command_queue = command_queue  # 初始化命令队列
        self.log_queue = log_queue          # 初始化日志队列
        self.initUI()  # 初始化用户界面

    def initUI(self):
        """
        设置主窗口的用户界面,包括关节控制的滑块、输入框、日志显示和姿态信息。
        """
        self.setGeometry(50, 50, 600, 800)  # 设置窗口位置和大小
        self.setWindowTitle('🔧 UR3机器人控制面板')  # 设置窗口标题

        main_layout = QVBoxLayout()  # 创建垂直布局,用于堆叠各个部分的控件

        self.labels = []       # 关节标签列表
        self.sliders = []      # 滑块控件列表
        self.spin_boxes = []   # 输入框控件列表
        self.angle_labels = [] # 显示当前角度的标签列表

        # 创建六个关节的控制界面
        for i in range(1, 7):
            h_layout = QHBoxLayout()  # 创建水平布局,用于放置单个关节的控件

            # 创建并添加关节标签
            label = QLabel(f'关节 {i} 角度 (°):')
            self.labels.append(label)

            # 创建并配置滑块控件
            slider = QSlider(Qt.Horizontal, self)
            slider.setMinimum(-90)  # 设置最小值
            slider.setMaximum(90)   # 设置最大值
            slider.setValue(0)      # 初始值为0
            slider.setTickPosition(QSlider.TicksBelow)  # 显示刻度
            slider.setTickInterval(15)  # 刻度间隔为15度
            # 连接滑块的值变化信号到处理函数,使用lambda确保传递正确的关节编号
            slider.valueChanged.connect(lambda value, idx=i: self.on_slider_change(idx, value))
            self.sliders.append(slider)

            # 创建并配置输入框控件
            spin_box = QSpinBox(self)
            spin_box.setMinimum(-90)  # 设置最小值
            spin_box.setMaximum(90)   # 设置最大值
            spin_box.setValue(0)      # 初始值为0
            # 连接输入框的值变化信号到处理函数
            spin_box.valueChanged.connect(lambda value, idx=i: self.on_spin_change(idx, value))
            self.spin_boxes.append(spin_box)

            # 创建并添加显示角度值的标签
            angle_label = QLabel("0")
            self.angle_labels.append(angle_label)

            # 将控件添加到水平布局中
            h_layout.addWidget(label)
            h_layout.addWidget(slider)
            h_layout.addWidget(spin_box)
            h_layout.addWidget(angle_label)
            main_layout.addLayout(h_layout)  # 将水平布局添加到垂直布局中

        # 添加姿态信息显示
        self.pose_label = QLabel("当前姿态信息：")
        self.pose_label.setStyleSheet("font-weight: bold;")
        main_layout.addWidget(self.pose_label)

        # 添加日志显示区域
        log_label = QLabel("日志信息：")
        log_label.setStyleSheet("font-weight: bold;")
        main_layout.addWidget(log_label)
        self.log_text = QTextEdit(self)
        self.log_text.setReadOnly(True)
        main_layout.addWidget(self.log_text)

        self.setLayout(main_layout)  # 设置窗口的主布局
        self.show()  # 显示窗口

        # 启动定时器,定期更新日志和姿态信息
        self.timer = self.startTimer(50)  # 每100毫秒触发一次

    def timerEvent(self, event):
        """
        定时器事件处理函数,用于更新日志和姿态信息。

        参数:
            event (QTimerEvent): 定时器事件对象。
        """
        # 获取并显示所有日志消息
        while not self.log_queue.empty():
            try:
                message = self.log_queue.get_nowait()
                if "当前累积变换矩阵" in message:
                    # 更新姿态信息显示
                    self.pose_label.setText(message)
                else:
                    self.log_text.append(message)
            except queue.Empty:
                break

    def on_slider_change(self, axis, value):
        """
        当滑块值变化时,更新对应的输入框和角度标签,并发送更新指令给Visualizer。

        参数:
            axis (int): 关节轴编号。
            value (int): 新的角度值（度）。
        """
        # 阻止信号递归触发
        self.spin_boxes[axis-1].blockSignals(True)
        self.spin_boxes[axis-1].setValue(value)  # 更新输入框的值
        self.spin_boxes[axis-1].blockSignals(False)

        # 更新角度标签显示
        self.angle_labels[axis-1].setText(str(value))

        # 发送设定角度命令给Visualizer
        command = {'type': 'set_angle', 'axis': axis, 'angle': value}
        self.command_queue.put(command)

    def on_spin_change(self, axis, value):
        """
        当输入框值变化时,更新对应的滑块和角度标签,并发送更新指令给Visualizer。

        参数:
            axis (int): 关节轴编号。
            value (int): 新的角度值（度）。
        """
        # 阻止信号递归触发
        self.sliders[axis-1].blockSignals(True)
        self.sliders[axis-1].setValue(value)  # 更新滑块的值
        self.sliders[axis-1].blockSignals(False)

        # 更新角度标签显示
        self.angle_labels[axis-1].setText(str(value))

        # 发送设定角度命令给Visualizer
        command = {'type': 'set_angle', 'axis': axis, 'angle': value}
        self.command_queue.put(command)

    def update_pose(self, cumulative_T):
        """
        更新姿态信息显示。

        参数:
            cumulative_T (np.array): 当前累积的变换矩阵。
        """
        pose_info = f"当前累积变换矩阵：\n{cumulative_T}"
        self.pose_label.setText(pose_info)

# -------------------------------------
# 主可视化函数
# -------------------------------------
def main_visualizer(command_queue, log_queue):
    """
    启动RobotVisualizer进程,开始可视化和处理指令。

    参数:
        command_queue (multiprocessing.Queue): 用于接收指令的队列。
        log_queue (multiprocessing.Queue): 用于发送日志消息的队列。

    返回:
        None
    """
    visualizer = RobotVisualizer(command_queue)
    visualizer.log_queue = log_queue  # 将日志队列传递给Visualizer
    visualizer.start()
    visualizer.join()

# -------------------------------------
# 主函数
# -------------------------------------
def main():
    """
    主函数,设置多进程环境,启动Visualizer进程和PyQt5应用。
    """
    multiprocessing.set_start_method('spawn')

    # 创建用于进程间通信的队列
    command_queue = multiprocessing.Queue()
    log_queue = multiprocessing.Queue()

    # 启动RobotVisualizer进程,传递命令队列和日志队列
    visualizer_process = multiprocessing.Process(target=main_visualizer, args=(command_queue, log_queue))
    visualizer_process.start()

    # 启动PyQt5应用,创建并显示主窗口
    app = QApplication(sys.argv)
    mainWindow = MainWindow(command_queue, log_queue)
    ret = app.exec_()  # 进入事件循环

    # 当PyQt5应用退出时,发送退出命令并等待Visualizer进程结束
    command_queue.put({'type': 'exit'})
    visualizer_process.join()

    sys.exit(ret)  # 退出程序

# -------------------------------------
# 程序入口
# -------------------------------------
if __name__ == '__main__':
    main()
