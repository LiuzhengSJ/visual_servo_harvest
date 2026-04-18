#! /home/liu/anaconda3/envs/mmpose1/bin/python
# -*- coding: utf-8 -*-

import rospy,sys
sys.path.append('/home/liu/realman/src/detector/scripts')
# from yolov5_1.detect_tomato_classify import YOLOV5Classify
# from yolov5_1.detect_tomato0 import YOLOV5ShufflenetClassify
# import can

import cv2
from cv_bridge import CvBridge
import time
import copy

from math import pi
from yolov5.detect_tomato import YOLOV5
from mmpose.keypoint1 import Keypoints
# from utils.startcam import KinectAzure
# from utils.startcam import *
# from utils.pose import *
# from utils.orbbec import *
from utils.convert_2d_to_3d import *
# from utils.distant_pick_list import Determine_pick_list
from utils.pose_transform import *
from utils.pose_transform import pick_cam_ori
from utils.realsense import *
from rm_75_msgs.msg import *
from rm_75_srvs.srv import *
from std_msgs.msg import *
import os
import pyrealsense2 as rs

dir_path = os.path.dirname(os.path.abspath(__file__))
path_rgb = os.path.join(dir_path + str('/test/rgb'))
path_pcd = os.path.join(dir_path + str('/test/pcd'))

class ArmController:
    def __init__(self):
        # ros获取机械臂当前位姿
        self.arm_end_state_client = rospy.ServiceProxy("arm_end_state",ArmState)
        # 机械臂按给定位姿进行轨迹规划并运动
        self.arm_move_client = rospy.ServiceProxy("move_arm",MoveArm)
        # 机械臂运动到预设姿态，包括初始姿态，收集姿态等
        self.arm_move_joint_client = rospy.ServiceProxy("move_arm_joint",MoveArmJoint)
    #获取机械臂当前状态
    def get_arm_state(self,state_link_name):
        end_state = None
         # --------------------获取机械臂当前位姿-------------
        rospy.wait_for_service("arm_end_state")
        request_end_link = ArmStateRequest()
        request_end_link.link_name = state_link_name  # tool_link
        result_future_end_link = self.arm_end_state_client(request_end_link)
        # 等待直到获取机械臂末端位姿
        end_state = []
        end_state.append(result_future_end_link.link_state.position.x)
        end_state.append(result_future_end_link.link_state.position.y)
        end_state.append(result_future_end_link.link_state.position.z)
        end_state.append(result_future_end_link.link_state.orientation.x)
        end_state.append(result_future_end_link.link_state.orientation.y)
        end_state.append(result_future_end_link.link_state.orientation.z)
        #这里参考不同机械臂读取位姿的返回值
        return end_state
    # 根据预设姿态移动机械臂
    def move_joint(self,type):
        rospy.wait_for_service("move_arm_joint")
        request_target_joint = MoveArmJointRequest()
        request_target_joint.move_point = type
        result_future_target_joint = self.arm_move_joint_client(request_target_joint)
        arrive = result_future_target_joint.arrive
        return arrive
    def move_arm(self,move_link_name, base_position, base_ori,type):
     # ----------------控制机械臂靠近-------
        rospy.wait_for_service("move_arm")
        request_target_pose = MoveArmRequest()
        request_target_pose.move_type = type
        request_target_pose.target_pose.header.frame_id = move_link_name  # tool_link
        request_target_pose.target_pose.pose.position.x = base_position[0]
        request_target_pose.target_pose.pose.position.y = base_position[1]
        request_target_pose.target_pose.pose.position.z = base_position[2]
        request_target_pose.target_pose.pose.orientation.x = base_ori[0]
        request_target_pose.target_pose.pose.orientation.y = base_ori[1]
        request_target_pose.target_pose.pose.orientation.z = base_ori[2]
        request_target_pose.target_pose.pose.orientation.w = 0
        result_future_target_pose = self.arm_move_client(request_target_pose)
        arrive = result_future_target_pose.arrive
        return arrive
    
class DetectorController:
    def __init__(self):
        self.distant_detect_client = rospy.ServiceProxy('/detect_objects',DetectObjects)
        
        self.keypoint_detect_client = rospy.ServiceProxy('/detect_keypoints', DetectKeypoints)
        self.bridge = CvBridge()
    def detect_distant_crop(self,image):
        image_msg = self.bridge.cv2_to_imgmsg(image,encoding="bgr8")
        
        req = DetectObjectsRequest()
        req.image = image_msg
        try:
            response = self.distant_detect_client(req)
            result = []
            for box in response.boxes:
                result.append({
                    'xmin': box.xmin,
                    'ymin': box.ymin,
                    'xmax': box.xmax,
                    'ymax': box.ymax,
                    'class': box.cls
                })
            print("远景检测结果为{}".format(result))
# 远景检测返回示例
# results = [
#     {'xmin': 100, 'ymin': 200, 'xmax': 300, 'ymax': 400, 'class': '1'},
#     {'xmin': 500, 'ymin': 600, 'xmax': 700, 'ymax': 800, 'class': '2'}
# ]
            return result
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s", str(e))
            return None    
    
    def detect_keypoints(self,image):
        image_msg = self.bridge.cv2_to_imgmsg(image,encoding='bgr8')
        
        req = DetectKeypointsRequest()
        req.image = image_msg
        try:
            response = self.keypoint_detect_client(req)
            result = []
            for kp in response.keypoints:
                result.append({
                    'x': kp.x,
                    'y': kp.y,
                    'confidence': kp.confidence
                })
            print("关键点检测结果为{}".format(result))
# results = [
#     {'x': 100, 'y‘：100，’confidence':0.9},
#     {'x': 100, 'y‘：100，’confidence':0.9}
#     {'x': 100, 'y‘：100，’confidence':0.9}
# ]
            return result
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s", str(e))
            return None

class tomato_pick:
    def __init__(self):
        rospy.init_node('tomato_pick')
        self.detect = True
        # ----------------------
        # 末端手爪can通讯
        # self.bus = can.interface.Bus(bustype = 'slcan', channel='/dev/ttyACM0', bitrate = 500000)
        
        # self.open = can.Message(arbitration_id=0x141,
        #                             data=[0xA4, 0x00, 0xE8,0x03, 0x38,0xC1, 0x01, 0x00],
        #                             is_extended_id=False)
        # self.close = can.Message(arbitration_id=0x141,
        #                             data=[0xA4, 0x00, 0xE8,0x03, 0x18,0xE4, 0x03, 0x00],
        #                             is_extended_id=False)
        # 初始化相机
        self.camera = RealSense(json_file="/home/liu/realman/src/detector/scripts/utils/d405.json", n_serial='218622271290',depth_width=1280, depth_height=720, conf_thresh=None, preset=4)
        
        self.robot = ArmController()
        
        self.detector = DetectorController()
    def startpick(self):
        while self.detect:
            forward_arrive =False
            # 设置机械臂初始采摘位置
            forward_arrive = self.robot.move_joint(type="0")

            if forward_arrive:
                
                name = len(os.listdir(path_rgb))
                dis_rgb_path = os.path.join(path_rgb, str(name) + str("_distant") +'.jpg')
                dis_pcd_path = os.path.join(path_pcd, str(name) + str("_distant") + '.txt')
                close_rgb_path = os.path.join(path_rgb, str(name) + str("_close") + '.jpg')
                close_pcd_path = os.path.join(path_pcd, str(name) + str("_close") + '.txt')
                
                
                # 获取远景图像
                distant_color_image_data, distant_xyz_image = self.camera.get_frames()
                # 保存远景图片及点云信息liuy
                cv2.imwrite(dis_rgb_path,distant_color_image_data)
                # np.savetxt(dis_pcd_path,distant_xyz_image)             

                # 检测远景目标
                pick_list= self.detector.detect_distant_crop(distant_color_image_data)
                distant_depth = distant_xyz_image[:, 2]
                distant_mask = np.logical_or(distant_depth <= 0.1, distant_depth >= 0.65)
                distant_xyz_image[distant_mask, :] = [0, 0, 0]              
                if len(pick_list)!=0:
                    print(f"检测到{len(pick_list)}个番茄")
                    # 获取所有远景串番茄在当前相机坐标系下的近景拍摄位置 距离30cm
                    locations = converted_distant_2d_3d(pick_list, distant_xyz_image, distant_color_image_data.shape)
                    
                    # 目标相对相机的位姿
                    # print(locations)
                    if np.array(locations).size == 0:
                        print("远景转化失败")
                        continue
                else:
                    print("在远景中没有检测到串番茄")
                    continue

                camera_close_positions = []
                camera_close_rotations = [] 
                tomato_classifys = []
                #location = [200,300,500,2],location[0:3]为相机坐标系下的点坐标，location[-1]为番茄类别
                for i, location in enumerate(locations):
                    # 针对所有串番茄获得其近景拍摄姿态
                    crop_location = location[0:3]
                    print(f"crop_location:{crop_location}")
                    #这里的近景观测姿态是根据果实分类写死了来的，到时可以考虑将几个果实统一挂成一个朝向，或者自己写死一个旋转
                    #提取果实朝向分类，无锡演示期间注释
                    crop_class = location[-1]
                    # crop_class = 0
                    camera_close_rotation_com= Close_Rotation_Single(crop_class)
                    endstate = self.robot.get_arm_state('0')
                    print(f"机械臂末端的状态为{endstate}")
                    # 将所有远景图像中的目标串的近景拍摄位置从相机坐标系转化到机械臂基座坐标系下
                    base_close_position, base_close_ori = target_pose_to_base(crop_location, camera_close_rotation_com,endstate)
                    # base_close_ori = 
                    if base_close_position is None:
                        continue
                    camera_close_positions.append(base_close_position)
                    camera_close_rotations.append(base_close_ori)
                    tomato_classifys.append(crop_class)
                    
                # 前往近景检测目标番茄
                for i,camera_close_position in enumerate(camera_close_positions):
                    # forward_arrive = self.move_joint(type="0")
                    camera_close_rotation = camera_close_rotations[i]
                    print(f"前往第{i}个近景位置进行近景图像采集")
                    tomato_classify = tomato_classifys[i]
                    camera_arrive = False               
                    # camera_arrive = self.move_close(move_link_name='cam_link', 
                    #                             position=camera_close_position, rotation=camera_close_rotation)
                    print(f'运动目标点为{camera_close_position},姿态为{camera_close_rotation}')
                    camera_close_position[0]=camera_close_position[0]-0.1
                    camera_arrive = self.robot.move_arm('0',camera_close_position,camera_close_rotation,type="0")
                                                         
                    if camera_arrive == False:
                        # self.robot.move_joint(type="2")
                        print('近景观测位姿不可达')
                        continue    
                    else:
                        # 获取近景图像
                        
                        close_color_image_data, close_xyz_image = self.camera.get_frames()
                        #保存近景图像和点云
                        cv2.imwrite(close_rgb_path,close_color_image_data)
                        # np.savetxt(close_pcd_path,close_xyz_image)
                        # 获取到目标框
                        pose_results = self.detector.detect_keypoints(close_color_image_data)
                        Keypoints_list,Keypoints_index = converted_keypoint_2d_3d(pose_results,close_xyz_image,close_color_image_data.shape)
                        print(f"关键点三维坐标列表为:{Keypoints_list}")
                        
                        print(f"关键点置信度列表：{Keypoints_index }")
                        if sum(x!=0 for x in Keypoints_index)<=4:
                            print("关键点检测失败")
                            continue
                        # 相机坐标系下的采摘点和采摘姿态            
                        pick_position, pick_rotation = pick_cam_ori(Keypoints_list,Keypoints_index,tomato_classify)
                        print("在相机坐标系下的采摘位姿")
                        print(pick_position)
                        print(pick_rotation)
                        endstate = self.robot.get_arm_state('Arm_Tip')
                        base_pick_position,base_pick_rotation = target_pose_to_base(pick_position,pick_rotation,endstate)                    
                        pick = self.move_pick(move_link_name='2', 
                                            position=base_pick_position, rotation=base_pick_rotation)         
                        if pick==False:
                            print("采摘失败")
                            continue
                        else:
                            # self.robot.move_joint("1")
                            # self.robot.move_joint("2")
                            # self.robot.move_joint("1")
                            print("采摘成功")
                            continue
                            
    def move_pick(self,move_link_name,position,rotation):
        pre_arrive = False
        tool_pick_arrive =False
        back_arrive = False     
        position[0] = position[0]-200
        pre_pick_position =  copy.deepcopy(position)

        pre_pick_position[1]= pre_pick_position[1] - 50
        
        if pre_pick_position is not None:
            pre_arrive = self.robot.move_arm(move_link_name,pre_pick_position,rotation,type="0")
            if pre_arrive == True:
                print("到达预采摘位置")
                tool_pick_arrive = self.robot.move_arm(move_link_name,position,rotation,type="0")
                if tool_pick_arrive == True:
                    back_arrive = self.robot.move_arm(move_link_name,pre_pick_position,rotation,type="0")
                    print("到达采摘点")
                    if back_arrive:
                        print("返回预采摘位置")
                        return True
                    else:
                        print("退回预采摘位置失败")
                        return False
                else:
                    print("到达采摘位置失败")
                    return False
            else:
                print("到达预采摘位置失败")
                return False
        else:
            print("pick_position is None")
            return False
    

if __name__=='__main__':
    try:
        node = tomato_pick()
        node.startpick()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    pass                    
                            

