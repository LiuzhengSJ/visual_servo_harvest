import numpy as np
import pyrealsense2 as rs
import cv2
import time
import math
from ultralytics import YOLO
import pyaubo_sdk  
from pyaubo_agvc_sdk import RpcClient  
import threading
import signal
import sys
import os


ROBOT_IP = "192.168.192.100"  
ROBOT_PORT = 30004
M_PI = math.pi

AGV_IP = "192.168.192.100"  
AGV_PORT = 30104
AGV_SPEED_FORWARD = 0  
AGV_SPEED_STOP = 0.0     
AGV_RUN_DISTANCE = 5.0


TOTAL_DURATION = 300
AGV_STOP_TIMEOUT = 10
YOLO_MODEL_PATH = "best.pt"  

# 放置位置
place_positions = [
    [0.039444,0.005934,2.324255,0.970927,1.437628,0.051138],
    [0.415214,0.006109,2.324255,0.970927,1.501856,0.051138],
    [-0.109, -0.688, 0.4283, 2.286, -0.087, 0.021],
    [-0.206, -0.661, 0.421, 2.299, -0.107, -0.104],
    [-0.274, -0.664, 0.429, 2.281, -0.108, -0.210],
    [0.1, 0.7, 0.5, 1.5708, 1.5708, 0],
    [0.2, 0.7, 0.5, 1.5708, 1.5708, 0],
    [0.3, 0.7, 0.5, 1.5708, 1.5708, 0],
    [0.4, 0.7, 0.5, 1.5708, 1.5708, 0],
    [0.5, 0.7, 0.5, 1.5708, 1.5708, 0]
]  
running = threading.Event()
has_tomato = threading.Event()
picking_done = threading.Event()
start_time = 0
agv_stop_time = 0
   #矩阵
R_tc = np.array([
    [-0.9996, -0.0270, -0.0103],
    [0.0267, -0.7993, 0.0245],
    [-0.0109, 0.0242, 0.7996]
])
T_tc = np.array([0, 0, 0])  

ui_callback = lambda x: None  

  # 机械臂RPC客户端
robot_rpc_client = pyaubo_sdk.RpcClient()

POSITION_TOLERANCE = 0.01  
ORIENTATION_TOLERANCE = 0.05  
JOINT_TOLERANCE = 0.02  


def set_ui_callback(callback):
    """设置UI回调函数（由main调用）"""
    global ui_callback
    if callable(callback):  
        ui_callback = callback
        print("UI回调函数已设置")
    else:
        print("警告：传入的UI回调不是可调用函数，使用默认空回调")

def exampleState(robot_name):
    joint_positions = robot_rpc_client.getRobotInterface(robot_name).getRobotState().getJointPositions()
    print("机械臂关节角度:", joint_positions)
    return joint_positions

def get_robot_end_effector_pose(robot_name):
    pose = robot_rpc_client.getRobotInterface(robot_name).getRobotState().getTcpPose()
    print("机械臂末端位姿:", pose)
    return pose

def exampleInverseK(robot_name, pose, reference_q):
    print("逆解计算:", pose)
    if not isinstance(pose, list) or len(pose) != 6:
        print(f"目标位姿格式错误，需要包含6个元素的列表: {pose}")
        return None, None
    waypoint_0_q = reference_q
    try:
        res = robot_rpc_client.getRobotInterface(robot_name).getRobotAlgorithm().inverseKinematics(waypoint_0_q, pose)
        print("逆解函数返回值:", res[1])
        print("逆解后的关节角:", res[0])
        if res[1] != 0:
            print(f"逆解失败，错误码: {res[1]}")
            return None, res[1]
        return res[0], res[1]
    except Exception as e:
        print(f"逆运动学计算错误: {e}")
        return None, None
    
def exampleStartup():
    robot_name = robot_rpc_client.getRobotNames()[0]
    if 0 == robot_rpc_client.getRobotInterface(robot_name).getRobotManage().poweron():
        print("The robot is requesting power-on!")
        if 0 == robot_rpc_client.getRobotInterface(robot_name).getRobotManage().startup():
            print("The robot is requesting startup!")
            while 1:
                robot_mode = robot_rpc_client.getRobotInterface(robot_name).getRobotState().getRobotModeType()
                print("Robot current mode: %s" % (robot_mode.name))
                if robot_mode == pyaubo_sdk.RobotModeType.Running:
                    break
                time.sleep(2)
            return robot_name
    return None
   #转换
def quaternion_to_rotation_matrix(q):
    w, x, y, z = q
    R = np.array([
        [1 - 2 * y**2 - 2 * z**2, 2 * x * y - 2 * w * z, 2 * x * z + 2 * w * y],
        [2 * x * y + 2 * w * z, 1 - 2 * x**2 - 2 * z**2, 2 * y * z - 2 * w * x],
        [2 * x * z - 2 * w * y, 2 * y * z + 2 * w * x, 1 - 2 * x**2 - 2 * y**2]
    ])
    return 

def euler_to_rotation_matrix(roll, pitch, yaw):
    R_x = np.array([[1, 0, 0],
                    [0, math.cos(roll), -math.sin(roll)],
                    [0, math.sin(roll), math.cos(roll)]])

    R_y = np.array([[math.cos(pitch), 0, math.sin(pitch)],
                    [0, 1, 0],
                    [-math.sin(pitch), 0, math.cos(pitch)]])

    R_z = np.array([[math.cos(yaw), -math.sin(yaw), 0],
                    [math.sin(yaw), math.cos(yaw), 0],
                    [0, 0, 1]])

    R = R_z @ R_y @ R_x
    return R

def get_robot_end_matrix(robot_name):
    try:
        pose = robot_rpc_client.getRobotInterface(robot_name).getRobotState().getTcpPose()
        pos = pose[:3]
        if len(pose) >= 7:
            quat = pose[3:7]
            R = quaternion_to_rotation_matrix(quat)
        elif len(pose) == 6:
            roll, pitch, yaw = pose[3:]
            R = euler_to_rotation_matrix(roll, pitch, yaw)
        else:
            print(f"位姿数据格式错误: {pose}")
            return np.eye(4)
        T = np.eye(4)
        T[:3, :3] = R
        T[:3, 3] = pos
        return T
    except Exception as e:
        print(f"获取机械臂末端位姿出错: {e}")
        return np.eye(4)
    
def camera_to_base(point_cam, robot_name):
    try:
        T_tool_base = get_robot_end_matrix(robot_name)
        T_camera_tool = np.eye(4)
        T_camera_tool[:3, :3] = R_tc
        T_camera_tool[:3, 3] = T_tc
        point_hom = np.append(point_cam, 1)
        point_base_hom = T_tool_base @ T_camera_tool @ point_hom
        return point_base_hom[:3]
    except Exception as e:
        print(f"坐标转换出错: {e}")
        return None
    
def check_joint_position(robot_name, target_joints):
    """校验当前关节位置是否达到目标位置"""
    current_joints = exampleState(robot_name)
    if not current_joints or len(current_joints) != len(target_joints):
        print("无法获取关节位置或关节数量不匹配，校验失败")
        return False
    errors = [abs(current - target) for current, target in zip(current_joints, target_joints)]
    max_error = max(errors)
    print(f"关节位置误差: 最大={max_error:.4f}rad, 阈值={JOINT_TOLERANCE:.4f}rad")
    return max_error < JOINT_TOLERANCE

def check_tcp_pose(robot_name, target_pose):
    """校验当前TCP位姿是否达到目标位姿"""
    current_pose = get_robot_end_effector_pose(robot_name)
    if not current_pose or len(current_pose) != 6:
        print("无法获取TCP位姿或位姿格式错误，校验失败")
        return False
    pos_error = np.linalg.norm(np.array(current_pose[:3]) - np.array(target_pose[:3]))
    ori_error = np.linalg.norm(np.array(current_pose[3:]) - np.array(target_pose[3:]))
    print(f"TCP位姿误差: 位置={pos_error:.4f}m, 姿态={ori_error:.4f}rad")
    print(f"误差阈值: 位置={POSITION_TOLERANCE:.4f}m, 姿态={ORIENTATION_TOLERANCE:.4f}rad")
    return pos_error < POSITION_TOLERANCE and ori_error < ORIENTATION_TOLERANCE

def waitArrival(impl, target_joints=None, target_pose=None):
    """
    等待运动完成并校验是否到达目标点
    :param impl: 机械臂接口实例
    :param target_joints: 目标关节位置（可选）
    :param target_pose: 目标TCP位姿（可选）
    :return: 0=成功, -1=失败, -2=超时
    """
    cnt = 0
    while impl.getMotionControl().getExecId() == -1:
        cnt += 1
        if cnt > 5:
            print("Motion fail! 获取执行ID超时")
            return -1
        time.sleep(0.5)
        print("getExecId: ", impl.getMotionControl().getExecId())
    
    id = impl.getMotionControl().getExecId()
    timeout = 0
    max_timeout = 30  # 最大等待时间（秒）
    
    # 等待运动完成
    while True:
        id1 = impl.getMotionControl().getExecId()
        if id != id1:
            break
        
        timeout += 0.5
        if timeout > max_timeout:
            print(f"运动超时{max_timeout}秒，强制退出")
            return -2
        
        time.sleep(0.5)
    
    # 运动完成后校验目标位置
    robot_name = robot_rpc_client.getRobotNames()[0]
    if target_joints is not None:
        if not check_joint_position(robot_name, target_joints):
            print("警告：关节位置未达到目标值！")
    elif target_pose is not None:
        if not check_tcp_pose(robot_name, target_pose):
            print("警告：TCP位姿未达到目标值！")
    
    print("运动完成并通过位置校验")
    return 0

def control_tool_io(robot_name, io_index, state):
    robot_rpc_client.getRobotInterface(robot_name).getIoControl().setToolDigitalOutput(io_index, state)
    output = robot_rpc_client.getRobotInterface(robot_name).getIoControl().getToolDigitalOutput(io_index)
    print(f"TOOL_IO[{io_index}] 设置为 {'接通' if state else '闭合'}，当前状态为：{'接通' if output else '闭合'}")

def create_placement_manager():
    current_index = 0
    def get_next_position():
        nonlocal current_index
        if not place_positions:
            print("警告：放置位置列表为空，使用默认位置")
            return [0.1, 0.7, 0.5, 1.5708, 1.5708, 0], 0  # 紧急默认位置
        position = place_positions[current_index]
        current_index = (current_index + 1) % len(place_positions)
        return position, current_index
    def get_current_index():
        return current_index
    return get_next_position, get_current_index

get_next_placement, get_placement_index = create_placement_manager()

# 新增：通用返回Home位置函数
def return_to_home(robot_name):
    """通用返回Home位置函数，异常时调用"""
    home_position = [ -1.52419,0.353080,1.870993,1.529083,1.511630,0]
    try:
        print("触发异常处理：返回Home位置")
        current_joints = exampleState(robot_name)
        if not current_joints:
            current_joints = home_position  # 使用home位置作为参考
        
        q_solution, status = exampleInverseK(robot_name, home_position, current_joints)
        if q_solution is None:
            q_solution = home_position  
        
        # 移动到Home位置
        robot_rpc_client.getRuntimeMachine().start()
        robot_rpc_client.getRobotInterface(robot_name).getMotionControl() \
            .moveJoint(q_solution, 180 * (M_PI / 180), 150* (M_PI / 180), 0, 0)
        result = waitArrival(robot_rpc_client.getRobotInterface(robot_name), target_joints=q_solution)
        if result != 0:
            print(f"返回Home位置失败，错误码: {result}")
        robot_rpc_client.getRuntimeMachine().stop()
        print("已成功返回Home位置")
    except Exception as e:
        print(f"返回Home位置失败: {e}")

def control_robot(robot_name, pose, angle_rad):
    home_position = [ -1.52419,0.353080,1.870993,1.529083,1.511630,0]
    current_joints = exampleState(robot_name)
    if not current_joints:
        print("获取关节位置失败，终止抓取并返回Home")
        return_to_home(robot_name)  # 关节失败返回Home
        return
    print(f"\n处理目标: {pose}")

    target_pose = pose.copy()
    target_pose[1] += 0.15
    print(f"目标抓取位姿: {target_pose}")
    
    # 移动到抓取位姿
    q_solution, status = exampleInverseK(robot_name, target_pose, current_joints)
    if q_solution is None:  # 逆解失败返回Home
        print("逆解失败，返回Home位置")
        return_to_home(robot_name)
        return
    
    robot_rpc_client.getRuntimeMachine().start()
    robot_rpc_client.getRobotInterface(robot_name).getMotionControl() \
        .moveJoint(q_solution, 180 * (M_PI / 180), 150* (M_PI / 180), 0, 0)

    result = waitArrival(robot_rpc_client.getRobotInterface(robot_name), target_joints=q_solution, target_pose=target_pose)
    if result != 0:
        print(f"到达抓取位置失败，错误码: {result}，返回Home")
        robot_rpc_client.getRuntimeMachine().stop()
        return_to_home(robot_name)
        return
    robot_rpc_client.getRuntimeMachine().stop()
    current_joints = q_solution 
    print("已到达抓取位置")
    time.sleep(0.5)

    target_pose1 = pose.copy()
    target_pose1[2] += 0.13  # 上升12cmq

    print(f"目标抓取位姿: {target_pose}")
    
    # 移动到抓取位姿
    q_solution1, status = exampleInverseK(robot_name, target_pose1, current_joints)
    if q_solution1 is None:  # 返回Home
        print("逆解失败，返回Home位置")
        return_to_home(robot_name)
        return
    
    robot_rpc_client.getRuntimeMachine().start()
    robot_rpc_client.getRobotInterface(robot_name).getMotionControl() \
        .moveJoint(q_solution1, 180 * (M_PI / 180), 150* (M_PI / 180), 0, 0)
    # 等待并校验抓取位置
    result = waitArrival(robot_rpc_client.getRobotInterface(robot_name), target_joints=q_solution1, target_pose=target_pose1)
    if result != 0:
        print(f"到达抓取位置失败，错误码: {result}，返回Home")
        robot_rpc_client.getRuntimeMachine().stop()
        return_to_home(robot_name)
        return
    robot_rpc_client.getRuntimeMachine().stop()
    current_joints = q_solution  # 更新关节位置
    print("已到达抓取位置")
    # 夹爪闭合（抓取）
    control_tool_io(robot_name, 1, True)
    time.sleep(2)
    control_tool_io(robot_name, 1, False)
    # 获取下一个放置位置
    current_place_pos, next_index = get_next_placement()
    print(f"移动到放置位置 {next_index}: {current_place_pos}")

    robot_rpc_client.getRuntimeMachine().start()
    robot_rpc_client.getRobotInterface(robot_name).getMotionControl() \
                .moveJoint(home_position, 180 * (M_PI / 180), 150* (M_PI / 180), 0, 0)
    # 校验Home位置
    result = waitArrival(robot_rpc_client.getRobotInterface(robot_name), target_joints=home_position)
    if result != 0:
        print(f"返回Home位置失败，错误码: {result}")
    robot_rpc_client.getRuntimeMachine().stop()

    robot_rpc_client.getRuntimeMachine().stop()

    print("返回home位置完成")
    time.sleep(2)

    
    print("返回中间点1位置完成")
            
    # 移动到放置位置
    q_lay, status = exampleInverseK(robot_name, current_place_pos, current_joints)
    if q_lay is None:
        print("逆解失败，无法移动到放置位置，返回Home")
        return_to_home(robot_name)  # 逆解失败返回Home
        return
    try:
        robot_rpc_client.getRuntimeMachine().start()
        robot_rpc_client.getRobotInterface(robot_name).getMotionControl() \
            .moveJoint(q_lay, 180 * (M_PI / 180), 150* (M_PI / 180), 0, 0)
        # 验放置位置
        result = waitArrival(robot_rpc_client.getRobotInterface(robot_name), target_joints=q_lay, target_pose=current_place_pos)
        if result != 0:
            print(f"到达放置位置失败，错误码: {result}，返回Home")
            robot_rpc_client.getRuntimeMachine().stop()
            return_to_home(robot_name)
            return
        robot_rpc_client.getRuntimeMachine().stop()
    except Exception as e:
        print(f"机械臂运动失败: {e}，返回Home位置")
        robot_rpc_client.getRuntimeMachine().stop()
        return_to_home(robot_name)  # 运动失败返回Home
        return
    current_joints = q_lay  # 更新关节位置
    print(f"已到达放置位置 {next_index}")
    control_tool_io(robot_name, 0, True)
    time.sleep(2)  # 模拟抓取时间
    control_tool_io(robot_name, 0, False)

    # 返回home位置
    print("返回home位置...")
    try:
        robot_rpc_client.getRuntimeMachine().start()
        robot_rpc_client.getRobotInterface(robot_name).getMotionControl() \
                .moveJoint(home_position, 180 * (M_PI / 180), 150* (M_PI / 180), 0, 0)
        # 校验Home位置
        result = waitArrival(robot_rpc_client.getRobotInterface(robot_name), target_joints=home_position)
        if result != 0:
            print(f"返回Home位置失败，错误码: {result}")
        robot_rpc_client.getRuntimeMachine().stop()
    except Exception as e:
        print(f"返回Home位置失败: {e}")
        robot_rpc_client.getRuntimeMachine().stop()
        return_to_home(robot_name)  # 返回Home
        return
    print("返回home位置完成")
    time.sleep(2)

def init_camera():
    """初始化相机，增加异常捕获"""
    try:
        pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        profile = pipeline.start(config)
        
        depth_sensor = profile.get_device().first_depth_sensor()
        depth_scale = depth_sensor.get_depth_scale()
        depth_profile = rs.video_stream_profile(profile.get_stream(rs.stream.depth))
        depth_intrinsics = depth_profile.get_intrinsics()
        
        align_to = rs.stream.color
        align = rs.align(align_to)
        
        print(f"相机初始化完成 | 深度比例尺: {depth_scale}")
        return pipeline, align, depth_intrinsics
    except Exception as e:
        print(f"相机初始化失败: {e}")
        raise  # 主程序处理

def init_tomato_detector(model_path=None):
    """初始化YOLO模型，增加路径校验和异常捕获"""
    use_path = model_path or YOLO_MODEL_PATH or "best.pt"
    print(f"加载YOLO模型 | 路径: {use_path}")
    
    # 校验路径是否存在
    if not os.path.exists(use_path):
        raise FileNotFoundError(f"YOLO模型文件不存在: {use_path}")
    
    # 校验文件后缀是否正确
    if not use_path.endswith(".pt"):
        raise TypeError(f"模型文件格式错误: {use_path}，仅支持.pt格式")
    
    try:
        model = YOLO(use_path, task="detect")  # 显式指定任务为检测，避免警告
        print("YOLO模型加载成功")
        return model
    except Exception as e:
        print(f"YOLO模型加载失败: {e}")
        raise  # 主程序处理

def filter_ripe_tomatoes(results, confidence_threshold=0.8):
    ripe_tomatoes = []
    for box in results.boxes:
        if int(box.cls) == 0 and box.conf >= confidence_threshold:
            ripe_tomatoes.append(box)
    return ripe_tomatoes

def draw_tomato_tilt_line(color_image, x1, y1, x2, y2, pixel_x):
    roi = color_image[y1:y2, x1:x2]
    gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
    edges = cv2.Canny(gray, 50, 150)

    lines = cv2.HoughLinesP(edges, 1, np.pi / 180, threshold=50, minLineLength=50, maxLineGap=10)
    angle = 0

    if lines is not None:
        longest_line = max(lines, key=lambda line: np.linalg.norm(np.array(line[0][:2]) - np.array(line[0][2:])))
        x1_line, y1_line, x2_line, y2_line = longest_line[0]
        x1_line += x1
        x2_line += x1
        y1_line += y1
        y2_line += y1

        vector_tilt = np.array([x2_line - x1_line, y2_line - y1_line])
        vector_vertical = np.array([0, y2 - y1])
        dot_product = np.dot(vector_tilt, vector_vertical)
        norm_tilt = np.linalg.norm(vector_tilt)
        norm_vertical = np.linalg.norm(vector_vertical)
        
        if norm_tilt > 0 and norm_vertical > 0:
            cos_angle = dot_product / (norm_tilt * norm_vertical)
            cos_angle = max(-1.0, min(1.0, cos_angle))  # 防止数值溢出导致arccos错误
            angle = np.arccos(cos_angle) * 180 / np.pi
            angle = angle if x2_line > x1_line else -angle
            angle = max(-30, min(30, angle))


        cv2.putText(color_image, f"Angle: {angle:.2f}°", (x1, y2 + 20), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        print(f"检测到的番茄串倾角为: {angle:.2f}°")

    return angle

def fit_red_line(crop_image, x1, y1):
    lower_red1 = np.array([0, 120, 70])
    upper_red1 = np.array([10, 255, 255])
    lower_red2 = np.array([160, 120, 70])
    upper_red2 = np.array([180, 255, 255])

    hsv = cv2.cvtColor(crop_image, cv2.COLOR_BGR2HSV)
    mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
    mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
    mask = cv2.bitwise_or(mask1, mask2)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return None

    all_points = np.vstack(contours).squeeze()
    if len(all_points) < 2:
        return None

    [vx, vy, x, y] = cv2.fitLine(all_points, cv2.DIST_L2, 0, 0.01, 0.01)
    lefty = int((-x * vy / vx) + y) if vx != 0 else y1 
    righty = int(((crop_image.shape[1] - x) * vy / vx) + y) if vx != 0 else y1 + crop_image.shape[0]

    p1 = (x1, y1 + lefty)
    p2 = (x1 + crop_image.shape[1], y1 + righty)
    return p1, p2


def vision_detection_thread(pipeline, align, depth_intrinsics, model, robot_name):
    global start_time, agv_stop_time
    start_time = time.time()
    print("视觉检测线程启动 | 开始检测番茄...")
    
    while running.is_set():
        # 超时检查
        if time.time() - start_time > TOTAL_DURATION:
            print("视觉检测：总时长已到，准备停止...")
            running.clear()
            break
        
        # 获取相机帧
        try:
            frames = pipeline.wait_for_frames(timeout_ms=5000)  # 超时1秒，避免阻塞
            if not frames:
                print("未获取到相机帧，重试...")
                time.sleep(0.1)
                continue
            aligned_frames = align.process(frames)
            depth_frame = aligned_frames.get_depth_frame()
            color_frame = aligned_frames.get_color_frame()
            if not depth_frame or not color_frame:
                print("深度帧或彩色帧为空，重试...")

                empty_frame = np.zeros((480, 640, 3), dtype=np.uint8)
                ui_callback(empty_frame)
                time.sleep(0.1)
                continue
            
            color_image = np.asanyarray(color_frame.get_data())
            annotated_image = color_image.copy()
            
        except Exception as e:
            print(f"获取相机帧失败: {e}")

            error_frame = np.zeros((480, 640, 3), dtype=np.uint8)
            ui_callback(error_frame)
            time.sleep(0.1)
            continue
        
        # 模型检测
        try:
            if has_tomato.is_set() and not picking_done.is_set():
                # AGV停止
                if agv_stop_time == 0:
                    agv_stop_time = time.time()
                    print(f"AGV已停止，等待2秒后开始详细检测...")
                    ui_callback(annotated_image)  # 直接传递原始画面
                    time.sleep(2)
                    continue
                
                # 超时检查
                if time.time() - agv_stop_time > AGV_STOP_TIMEOUT:
                    print(f"AGV停止超时{AGV_STOP_TIMEOUT}秒，继续前进")
                    has_tomato.clear()
                    picking_done.clear()
                    agv_stop_time = 0
                    ui_callback(annotated_image)
                    time.sleep(0.1)
                    continue

                # 执行检测
                results = model(annotated_image, classes=0, conf=0.5, verbose=False)
                annotated_image = results[0].plot()  # YOLO自动绘制目标框
                ripe_tomatoes = filter_ripe_tomatoes(results[0], confidence_threshold=0.8)
                
                if len(ripe_tomatoes) > 0:
                    tomato_count = 0
                    robot_end_effector_pose = get_robot_end_effector_pose(robot_name)
                    if not robot_end_effector_pose:
                        print("获取末端位姿失败，跳过抓取并返回Home")
                        return_to_home(robot_name)  # 获取末端位姿失败返回Home
                        ui_callback(annotated_image)
                        time.sleep(0.1)
                        continue
                    
                    for box in ripe_tomatoes:
                        x1, y1, x2, y2 = box.xyxy[0].cpu().numpy().astype(int)
                        pixel_x = int((x1 + x2) / 2)
                        pixel_y = int((y1 + y2) / 2)

                        try:
                            depth = depth_frame.get_distance(pixel_x, pixel_y)
                            if depth <= 0.01:  # 过滤过近/无效深度
                                print(f"无效深度值 ({depth:.3f}m) at ({pixel_x}, {pixel_y})")
                                continue

                            # 坐标转换与目标标注
                            point_cam = rs.rs2_deproject_pixel_to_point(depth_intrinsics, [pixel_x, pixel_y], depth)
                            point_base = camera_to_base(point_cam, robot_name)
                            if point_base is None:
                                print(f"坐标转换失败 for 像素点 ({pixel_x}, {pixel_y})，返回Home")
                                return_to_home(robot_name)  # 坐标转换失败返回Home
                                continue

                            # 绘制倾角
                            angle = draw_tomato_tilt_line(annotated_image, x1, y1, x2, y2, pixel_x)
                            angle_rad = math.radians(angle)
                            point_robot_pose = list(point_base) + robot_end_effector_pose[3:]
                            tomato_count += 1

                            # 绘制目标核心信息
                            cv2.circle(annotated_image, (pixel_x, pixel_y), 5, (0, 0, 255), -1)  # 中心点
                            cv2.line(annotated_image, (pixel_x, y1), (pixel_x, y2), (255, 0, 0), 2)  # 中线
                            crop_image = annotated_image[y1:y2, x1:x2]
                            line_points = fit_red_line(crop_image, x1, y1)
                            if line_points:
                                p1, p2 = line_points
                                cv2.line(annotated_image, p1, p2, (255, 0, 0), 2)  # 红线拟合
                            # 目标深度信息
                            cv2.putText(annotated_image, f"Depth: {depth:.3f}m",
                                        (x1, y1 - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

                            print(f"检测到目标 #{tomato_count}:")
                            print(f"  像素坐标: ({pixel_x}, {pixel_y})")
                            print(f"  相机坐标: ({point_cam[0]:.3f}, {point_cam[1]:.3f}, {point_cam[2]:.3f})m")
                            print(f"  机械臂基坐标: ({point_base[0]:.3f}, {point_base[1]:.3f}, {point_base[2]:.3f})m")

                            # 执行抓取
                            control_robot(robot_name, point_robot_pose, angle_rad)
                            picking_done.set()
                            
                            # 重置状态
                            has_tomato.clear()
                            agv_stop_time = 0
                            break

                        except Exception as e:
                            print(f"处理目标时出错: {str(e)}，返回Home位置")
                            return_to_home(robot_name)  #处理目标异常返回Home
                            continue
            else:
                # AGV移动：快速检测
                results = model(annotated_image, classes=0, conf=0.5, verbose=False)
                annotated_image = results[0].plot()  
                ripe_tomatoes = filter_ripe_tomatoes(results[0], confidence_threshold=0.8)
                
                if len(ripe_tomatoes) > 0 and not picking_done.is_set():
                    has_tomato.set()
                    print(f"视觉检测：识别到 {len(ripe_tomatoes)} 个成熟番茄 | 触发AGV暂停")
                else:
                    if picking_done.is_set():
                        has_tomato.clear()
                        picking_done.clear()
                        agv_stop_time = 0
                        print("视觉检测：采摘完成 | 允许AGV继续前进")
        
        except Exception as e:
            print(f"检测逻辑出错: {e}，返回Home位置")
            return_to_home(robot_name)  # 检测逻辑异常返回Home

            pass
        
        remaining_time = max(0, int(TOTAL_DURATION - (time.time() - start_time)))
        # 1. 剩余时间
        cv2.putText(annotated_image, f"Time Left: {remaining_time}s", 
                    (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
        # 2. 番茄检测状态
        cv2.putText(annotated_image, f"Tomato: {'Yes' if has_tomato.is_set() else 'No'}", 
                    (10, 70), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
        # 3. 采摘状态
        cv2.putText(annotated_image, f"Picking: {'Yes' if picking_done.is_set() else 'No'}", 
                    (10, 110), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 0, 0), 2)

        
        # 传递最终画面到UI
        try:
            ui_callback(annotated_image)
        except Exception as e:
            print(f"UI回调失败: {e}")
        
        time.sleep(0.05)  # 控制帧率
    
    # 线程结束
    try:
        pipeline.stop()
        stop_frame = np.zeros((480, 640, 3), dtype=np.uint8)
        ui_callback(stop_frame)
    except Exception as e:
        print(f"释放相机失败: {e}")
    print("视觉检测线程停止")


def agv_control_thread(agv_client):
    print("AGV控制线程启动 | 等待视觉检测信号...")
    
    while running.is_set():
        try:
            # 超时检查
            if time.time() - start_time > TOTAL_DURATION:
                print("AGV控制：总时长已到，停止AGV...")
                agv_client.set_control_speed(AGV_SPEED_STOP, AGV_SPEED_STOP)
                running.clear()
                break
            
            # 控制逻辑
            if has_tomato.is_set() and not picking_done.is_set():
                # 停止AGV
                ret = agv_client.set_control_speed(AGV_SPEED_STOP, AGV_SPEED_STOP)
                if ret != 10100000:
                    print(f"AGV控制：停止指令失败（码{ret}），重新获取控制权...")
                    agv_client.release_priority()
                    time.sleep(0.5)
                    agv_client.set_priority("admin", AGV_IP)
            else:
                # 前进AGV
                ret = agv_client.set_control_speed(AGV_SPEED_FORWARD, AGV_SPEED_STOP)
                if ret != 10100000:
                    print(f"AGV控制：前进指令失败（码{ret}），重新获取控制权...")
                    agv_client.release_priority()
                    time.sleep(0.5)
                    agv_client.set_priority("admin", AGV_IP)
        
        except Exception as e:
            print(f"AGV控制线程出错: {e}")
            time.sleep(0.5)
            continue
        
        time.sleep(0.1)  # 100ms间隔
    
    # 线程结束
    try:
        agv_client.set_control_speed(AGV_SPEED_STOP, AGV_SPEED_STOP)
    except Exception as e:
        print(f"AGV停止失败: {e}")
    print("AGV控制线程停止 | AGV已停止")

def main():
    """主程序入口"""
    global robot_rpc_client
    print("="*50)
    print("开始初始化番茄采摘系统")
    print(f"当前参数：")
    print(f"  机械臂IP: {ROBOT_IP}:{ROBOT_PORT}")
    print(f"  AGV IP: {AGV_IP}:{AGV_PORT}")
    print(f"  YOLO模型: {YOLO_MODEL_PATH}")
    print(f"  总时长: {TOTAL_DURATION}s")
    print("="*50)
    
    # 1. 初始化机械臂
    try:
        robot_rpc_client.connect(ROBOT_IP, ROBOT_PORT)
        robot_rpc_client.setRequestTimeout(1000)
        if not robot_rpc_client.hasConnected():
            print("机械臂连接失败!")
            return

        print("机械臂连接成功!")
        robot_rpc_client.login("rob1", "123456")  
        
        if not robot_rpc_client.hasLogined():
            print("机械臂登录失败!")
            robot_rpc_client.disconnect()
            return

        print("机械臂登录成功!")
        
        # 启动机械臂
        robot_name = exampleStartup()
        if not robot_name:
            print("机械臂启动失败!")
            robot_rpc_client.disconnect()
            return

        print(f"机械臂 {robot_name} 启动成功!")
    except Exception as e:
        print(f"机械臂初始化失败: {e}")
        if robot_rpc_client.hasConnected():
            robot_rpc_client.disconnect()
        return

    # 2. 初始化AGV
    try:
        agv = RpcClient()  
        agv.setRequestTimeout(1000)
        print(f"\n正在连接AGV | IP: {AGV_IP}:{AGV_PORT}")
        
        connect_ret = agv.connect(AGV_IP, AGV_PORT)
        if connect_ret != 0:
            print(f"AGV连接失败（返回码{connect_ret}），程序退出")
            robot_rpc_client.disconnect()
            return
        
        login_ret = agv.login("admin", "admin") 
        if login_ret != 0:
            print(f"AGV登录失败（返回码{login_ret}），程序退出")
            agv.disconnect()
            robot_rpc_client.disconnect()
            return
        print("AGV连接与登录成功")
        
        agv.release_priority()
        time.sleep(0.5)
        control_ret = agv.set_priority("admin", AGV_IP)
        if control_ret != 10100000:
            print(f"AGV获取控制权失败（返回码{control_ret}），程序退出")
            agv.disconnect()
            robot_rpc_client.disconnect()
            return
        print("AGV控制权获取成功")
    except Exception as e:
        print(f"AGV初始化失败: {e}")
        robot_rpc_client.disconnect()
        return

    # 3. 初始化视觉
    try:
        print("\n初始化相机...")
        pipeline, align, depth_intrinsics = init_camera()
        
        print("初始化YOLO模型...")
        model = init_tomato_detector()
    except Exception as e:
        print(f"视觉初始化失败：{str(e)}，程序退出")
        agv.disconnect()
        robot_rpc_client.disconnect()
        return

    # 4. 启动线程
    print("\n启动核心线程...")
    running.set()
    vision_thread = threading.Thread(
        target=vision_detection_thread,
        args=(pipeline, align, depth_intrinsics, model, robot_name),
        daemon=True
    )
    agv_thread = threading.Thread(
        target=agv_control_thread,
        args=(agv,),
        daemon=True
    )
    
    vision_thread.start()
    agv_thread.start()
    print(f"\n所有线程启动完成 | 总运行时长：{TOTAL_DURATION}秒")

    # 5. 等待线程结束
    try:
        while running.is_set():
            time.sleep(1)
    except KeyboardInterrupt:
        print("\n用户手动中断程序...")
        running.clear()
        return_to_home(robot_name)
    finally:
        print("\n释放系统资源...")
        agv.release_priority()
        agv.disconnect()
        robot_rpc_client.disconnect()
        print("程序结束 | 所有设备已断开连接")

# 信号处理
def signal_handler(sig, frame):
    print(f"\n接收到信号 {sig}，正在终止程序并返回Home...")
    running.clear()

    try:
        robot_name = robot_rpc_client.getRobotNames()[0]
        return_to_home(robot_name)
    except:
        pass
    time.sleep(1)
    sys.exit(0)

if __name__ == "__main__":
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)
    main()