#! /home/hul/anaconda3/envs/torch/bin/python3
# -*- coding: utf-8 -*-
# 灰常棒 定时器凑合凑合可以用
# 继续干活活

import rospy,sys
sys.path.append('/home/nvidia/tomato_pick/src/detector/scripts')
from yolov5.detect_tomato_classify import YOLOV5Classify
from yolov5.detect_tomato0 import YOLOV5ShufflenetClassify
import can

import cv2
import time
import open3d as o3d
import copy


from yolov5.detect_tomato import YOLOV5
from mmpose.keypoint1 import Keypoints
from utils.startcam import KinectAzure
from utils.startcam import *
from utils.pose import *
from utils.orbbec import *
from utils.convert_2d_to_3d import *
# from utils.distant_pick_list import Determine_pick_list

from rm_75_msgs.msg import *
from rm_75_srvs.srv import *
from std_msgs.msg import *



# 循环发布数据，其中rospy.is_shutdown()的意思是只要节点关闭就返回true
# while not rospy.is_shutdown():

dir_path = os.path.dirname(os.path.abspath(__file__))
single_weight_path = os.path.join(dir_path + str('/yolov5/weights/distant/best.pt'))


config_file = os.path.join(dir_path + str('/mmpose/configs/tomato/hrnet_w32_h36m_256x256_pck.py'))
checkpoint_file = os.path.join(dir_path + str('/mmpose/tools/weights/hrnet_w32_h36m/best_EPE_epoch_215.pth'))
path_rgb = os.path.join(dir_path + str('test/rgb'))
path_pcd = os.path.join(dir_path + str('test/pcd'))



class Pick():
    def __init__(self):
        rospy.init_node('tomato_vision')

        self.detect = True
        # 设置关键点检测的目标框
        # self.pre_bbox = [600, 100, 1330, 1000]
        # self.pre_bbox = [600, 400, 1330, 1000]
        self.pre_bbox = [600, 350, 1330, 1000]

        # ----------------------
        # # 末端can通讯
        # self.bus = can.interface.Bus(bustype = 'slcan', channel='/dev/ttyACM0', bitrate = 500000)
        # self.open = can.Message(arbitration_id=0x141,
        #                             data=[164, 0, 232, 3, 232,110, 3, 0], 
        #                             is_extended_id=False)
        # self.close = can.Message(arbitration_id=0x141,
        #                             data=[164, 0, 144, 1, 0, 0, 0, 0], 
        #                             is_extended_id=False)



        # 初始化相机
        self.camera = OrbbecCam()


        # 初始化目标检测
        self.detector_distant = YOLOV5Classify(weights=single_weight_path, imgsz=640)  
        # 初始化关键点网络
        self.keypoint = Keypoints(config_file=config_file, checkpoint_file=checkpoint_file)  

        # 获取机械臂当前位姿
        self.arm_end_state_client = rospy.ServiceProxy("arm_end_state",current_location)
        # 机械臂按给定位姿进行轨迹规划并运动
        self.arm_move_client = rospy.ServiceProxy("move_arm",move_arm)
        # 机械臂运动到预设姿态，包括初始姿态，收集姿态等
        self.arm_move_joint_client = rospy.ServiceProxy("move_arm_joint",move_joint)

        # # *******************
        # # 升降柱移动
        # self.lift_set_height = rospy.Publisher("/rm_driver/Lift_SetHeight",Lift_Height,queue_size=10)
        
        # # 需通过先向机械臂发布信息
        # # 才能订阅升降状态
        # # 创建定时器，发布获取升降状态的话题"/rm_driver/Lift_GetState"
        # self.lift_get_height = rospy.Publisher("/rm_driver/Lift_GetState",Empty,queue_size=10)
        # self.lift_height = 0
        # self.pub_lift_state = False
        # self.timer = rospy.Timer(rospy.Duration(1),self.timerCallback)

    def startpick(self):
        while self.detect:
            forward_arrive =False
            # 设置机械臂初始采摘位置
            forward_arrive = self.move_joint(type="0")
            # *******************
            # # 设置升降柱初始高度
            # initial_lift = Lift_Height()
            # initial_lift.speed = 30
            # # initial_lift.height = 620
            # initial_lift.height = 690
            # self.lift_set_height.publish(initial_lift)
            # rospy.sleep(3.5)
            
            # ----------------------
            # 设置末端手爪张开
            # self.bus.send(self.open)

            if forward_arrive:
                
                name = len(os.listdir(path_rgb))
                dis_rgb_path = os.path.join(path_rgb, str(name) + str("_distant") +'.jpg')
                dis_pcd_path = os.path.join(path_pcd, str(name) + str("_distant") + '.txt')
                close_rgb_path = os.path.join(path_rgb, str(name) + str("_close") + '.jpg')
                close_pcd_path = os.path.join(path_pcd, str(name) + str("_close") + '.txt')
                
                # 获取远景图像
                distant_color_image_data, distant_xyz_image = self.camera.get_frames()
                # 保存远景图片及点云信息
                # cv2.imwrite(dis_rgb_path,distant_color_image_data)
                # np.savetxt(dis_pcd_path,distant_xyz_image)
                dist_image_shape = distant_color_image_data.shape
                
                # 对远景深度z过滤 0.1-0.8m
                distant_depth = distant_xyz_image[:, 2]
                distant_mask = np.logical_or(distant_depth <= 100, distant_depth >= 650)
                distant_xyz_image[distant_mask, :] = [0, 0, 0]

                # 检测远景目标
                pick_list, distant_image_detect = self.detector_distant.run_distant(distant_color_image_data)
                print(pick_list)

                if len(pick_list):
                    # 获取所有远景串番茄在当前相机坐标系下的近景拍摄位置 距离30cm
                    locations = converted_distant_2d_3d(pick_list, distant_xyz_image, dist_image_shape)
                    # 目标相对相机的位姿
                    # print(locations)
                    if np.array(locations).size == 0:
                        print("远景转化失败")
                        continue
                        # self.detect =False
                        # break
                else:
                    print("在远景中没有检测到串番茄")
                    continue
                    # self.detect =False
                    # break

                camera_close_positions = []
                camera_close_rotations = [] 
                tomato_classifys = []
                for i, location in enumerate(locations):
                    # 针对所有串番茄获得其近景拍摄姿态
                    camera_close_rotation_com= Close_Rotation_Single(location[-1])
                    # 将所有远景图像中的目标串的近景拍摄位置从相机坐标系转化到机械臂基座坐标系下
                    base_close_position, base_close_ori = self.pose_to_base('Arm_Tip', location[0: 3], camera_close_rotation_com)
                    if base_close_position is None:
                        continue
                    camera_close_positions.append(base_close_position)
                    camera_close_rotations.append(base_close_ori)
                    tomato_classifys.append(location[-1])

                # *******************
                # # 获取远景拍摄时机械臂基座的位置 distant_lift_height
                # self.pub_lift_state = True
                # rospy.sleep(2)
                # print('远景视角，机械臂基座高度：%f'%self.lift_height)
                # # 保存远景图像对应的机械臂基座的位置
                # distant_lift_height = copy.deepcopy(self.lift_height)
                # self.pub_lift_state = False       
   
                # 前往近景检测目标番茄
                for i,camera_close_position in enumerate(camera_close_positions):
                    # forward_arrive = self.move_joint(type="0")
                    
                    print("前往第{0}个近景位置进行近景图像采集".format(i))
                    print(camera_close_position)
                    tomato_classify = tomato_classifys[i]

                    # *******************
                    # # 升降柱当前高度
                    # self.pub_lift_state = True
                    # rospy.sleep(1)
                    # print('当前机械臂基座位置：%f'%self.lift_height)
                    # current_lift_height = copy.deepcopy(self.lift_height)
                    # self.pub_lift_state = False

                    # 根据当前目标与基座在x轴的位置关系 判断是否需要移动升降柱
                    # camera_close_position 远景检测时，目标串番茄在基座坐标系下的位置
                    # distant_lift_height 远景检测时升降柱的高度
                    # current_lift_height 当前升降柱高度
                    # lift_move_arrive = False
                    # lift_move_arrive, pose_height  = self.move_lift(camera_close_position[0],distant_lift_height,current_lift_height)
                    # # 升降无法移动，即无法到达合理采摘区域
                    # self.pub_lift_state = False

                    lift_move_arrive = True
                    if lift_move_arrive == False:
                        continue
                    else:
                    # ------------------------------------------------------------------------
                        # 更新目标串番茄在基座坐标系下的位置
                        # 机械臂运动到近景拍摄位置
                        # camera_close_position[0] = pose_height
                        camera_arrive = False               
                        camera_arrive = self.move_close(move_link_name='cam_link', 
                                                    position=camera_close_positions[i], rotation=camera_close_rotations[i])
                        
# -----------------------------------近景--------------------------------------------------                                        
                        if camera_arrive == False:
                            print('近景观测位姿不可达')
                            # forward_arrive = self.move_joint(type="0")
                            continue  
                            
                        else:
                            # 获取近景图像
                            close_color_image_data, close_xyz_image = self.camera.get_frames()

                            # 近景深度值过滤
                            close_depth = close_xyz_image[:, 2]
                            close_mask = np.logical_or(close_depth <= 100, close_depth >= 380)
                            close_xyz_image[close_mask, :] = [0, 0, 0]
                                
                            close_image_shape = close_color_image_data.shape
                            # 获取到目标框
                            pre_bbox = {'bbox': self.pre_bbox}
                            pose_results = self.keypoint.pose_detect(close_color_image_data, [pre_bbox])[0]["keypoints"]
                            print("关键点检测结果：")
                            print(pose_results)
                            
                            # 相机坐标系下的采摘点和采摘姿态            
                            pick_position, pick_rotation = Pick_cam_ori(pose_results, close_xyz_image,close_image_shape,tomato_classify)
                            print("在相机坐标系下的采摘姿态")
                            print(pick_position)
                            print(pick_rotation)
                            # source = o3d.geometry.PointCloud()
                            # source.points = o3d.utility.Vector3dVector(close_xyz_image)
                            # coord_frame0 = o3d.geometry.TriangleMesh.create_coordinate_frame(size=100)
                            # o3d.visualization.draw_geometries([source, coord_frame0])
                            # 判断采摘点/姿态是否存在
                            if (pick_position.any() and pick_rotation.any()):
                                # open3d采摘姿态
                                # source = o3d.geometry.PointCloud()
                                # source.points = o3d.utility.Vector3dVector(close_xyz_image)
                                # coord_frame0 = o3d.geometry.TriangleMesh.create_coordinate_frame(size=100)
                                # coord_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=100)
                                # coord_frame.translate(pick_position)
                                # coord_frame.rotate(pick_rotation)
                                # o3d.visualization.draw_geometries([source, coord_frame, coord_frame0])
                                
                                # 机械臂运动到采摘位置    
                                # ------------------------------------------------------------------------                        
                                pick = self.move_pick(arm_end_state_link_name='Arm_Tip',move_link_name='tool_link', 
                                                        position=pick_position, rotation=pick_rotation)
                                
                                                     
                                if pick==False:
                                    print("采摘失败")
                                    continue
                                else:
                                    collect_arrive = self.move_joint(type="1")
                                    # collect_lift = Lift_Height()
                                    # collect_lift.speed = 35
                                    # # collect_lift.height =360
                                    # collect_lift.height =400
                                    # self.lift_set_height.publish(collect_lift)
                                    
                                    # rospy.sleep(3)
                                    # if collect_arrive:
                                    #     # self.bus.send(self.open)
                                    #     rospy.sleep(3)
                                    #     print("采摘成功")
                                       
                                    #     self.move_joint(type="2")   
                                    # else:
                                    #     print("收集位置无法到达")
                                    continue
                                    

    def timerCallback(self,event):
        if self.pub_lift_state:
            lift_status_requests = Empty()
            self.lift_get_height.publish(lift_status_requests)
            rospy.sleep(0.5)
            lift_state = rospy.Subscriber('/rm_driver/LiftState', LiftState,callback =self.lift_state_callback, queue_size=1)
            rospy.sleep(0.1)

    def lift_state_callback(self,msg):
        self.lift_height = msg.height

    # 未考虑上下限   
    def move_lift(self,pose,distant_lift_height,current_lift_height):
        # 设置300为合理采摘高度
        Reasonable_height = 280
        new_lift = Lift_Height()
        new_lift.speed = 35
        # pose为远景机械臂基座和目标之间的高度差 
        # 判读远景机械臂位置与当前机械臂位置是否相同
        # 相同则基于远景位置运动升降
        if abs(distant_lift_height-current_lift_height)<=10:
            if 200<=pose<=300:
                return True,pose
            # 升降过高，需要下移
            elif pose<200:
                move_distance = Reasonable_height - pose
                # 升降可下降距离为lift 不能过限 lift 最低限制位置为180mm处
                Descent_height_limit = distant_lift_height -200
                if Descent_height_limit >= move_distance:
                    new_lift.height = int(distant_lift_height - move_distance)
                    self.lift_set_height.publish(new_lift)
                    # 等待升降执行
                    rospy.sleep(1.5)
                    return True, Reasonable_height
                # 超过限制，升降无法到达 
                else:
                    print('下降高度受限')
                    return False, pose
            # 升降过低，需要上移
            elif pose >300:
                move_distance = pose - Reasonable_height
                # print(move_distance)
                # 升降可升高距离为900 -lift 不能过限 lift 最高限制位置为900mm处
                Ascent_height_limit = 800 - distant_lift_height
                if Ascent_height_limit >= move_distance:
                    new_lift.height = int(distant_lift_height + move_distance)
                    self.lift_set_height.publish(new_lift)
                    # 等待升降执行
                    rospy.sleep(1.5)
                    return True, Reasonable_height
                # 超过限制，升降无法到达 
                else:
                    print('上升高度受限')
                    return False, pose
        else:
            target_pose = pose+distant_lift_height
            arm_target_pose = target_pose - Reasonable_height
            if 180<arm_target_pose<800:
                new_lift.height = int(arm_target_pose)
                self.lift_set_height.publish(new_lift)
                    # 等待升降执行
                rospy.sleep(1.5)
                return True,Reasonable_height
            else:
                print('升降移动受限')
                return False, pose
    
    def move_close(self, move_link_name, position, rotation):
        # link7  cam_link
        arrive = False
        arrive = self.move_arm(move_link_name,position/1000, rotation,type="0")
        print("机械臂是否到达近景观测位置：%s"%arrive)
        return arrive     
    
    def move_pick(self,arm_end_state_link_name, move_link_name, position, rotation):
        pre_arrive = False
        tool_pick_arrive =False
        back_arrive = False 

        # 采摘位置转换到基座坐标系下
        base_pick_position, base_pick_ori = self.pose_to_base(arm_end_state_link_name, position, rotation)
        print("目标相对基座坐标系的位置")
        print(base_pick_position)
        print(base_pick_ori)

        base_pick_position[1] = base_pick_position[1] - 8
       
        # # 预采摘位置距离目标番茄10cm
        base_pre_pick_position = copy.deepcopy(base_pick_position) 
        base_pre_pick_position[1] = base_pre_pick_position[1] -100
        
        if base_pick_position is None:
            return False
        else:
            pre_arrive = self.move_arm(move_link_name,base_pre_pick_position/1000, base_pick_ori,type="0")
            
            if pre_arrive:
                print("到达预采摘位置")
                tool_pick_arrive = self.move_arm(move_link_name,base_pick_position/1000, base_pick_ori,type="1")
                
                if tool_pick_arrive:
                    print("到达采摘位置")
                    # pick_arrive = True
                    # ----------------
                    # self.bus.send(self.close)
                    rospy.sleep(6)
                    base_pre_pick_position[0] = base_pre_pick_position[0] - 20
                    back_arrive = self.move_arm(move_link_name,base_pre_pick_position/1000, base_pick_ori,type="1")
                    if back_arrive:
                        print("返回预采摘位置")
                        # self.move_joint(type="0")
                        return True
                    else:
                        print('退回预采摘位置不可达')
                        return False
                else:
                    print('采摘位置不可达')
                    return False
            else:
                print('预采摘位置不可达')
                return False

            
    # 相机坐标转化为基座坐标系
    def pose_to_base(self,arm_end_link_name, position_to_cam, base_to_cam):
        # --------------------获取机械臂当前位姿-------------
        end_state = self.current_location(arm_end_link_name)
        # x y z rx ry rz
        if end_state is not None:
            # 基座坐标
            base_position, base_orientation = cam_to_base(position_to_cam,base_to_cam,end_state)
            return base_position, base_orientation
        else:
            return None,None

    # 末端位姿
    def current_location(self,state_link_name):
        end_state = None
         # --------------------获取机械臂当前位姿-------------
        rospy.wait_for_service("arm_end_state")
        request_end_link = ArmStateRequest()
        request_end_link.link_name = state_link_name  # tool_link
        result_future_end_link = self.arm_end_state_client(request_end_link)
        # 等待直到获取机械臂末端位姿
        end_state = result_future_end_link
        # x y z rx ry rz
        return end_state
   
    # 根据预设姿态移动机械臂
    def move_joint(self,type):
        rospy.wait_for_service("move_arm_joint")
        request_target_joint = MoveArmJointRequest()
        request_target_joint.move_point = type
        result_future_target_joint = self.arm_move_joint_client(request_target_joint)
        arrive = result_future_target_joint.arrive
        return arrive
    
    # 根据位置和姿态移动机械臂
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


if __name__ =='__main__':
    try:
        node = Pick()
        node.startpick()
        # node.stop()
        # keep node running，and check whether an exit command is received (Ctrl+C)
        rospy.spin()

    except rospy.ROSInterruptException:
        pass
    pass
