#! /home/hul/anaconda3/envs/torch/bin/python3
# -*- coding: utf-8 -*-
import rospy,sys
# sys.path.append('/home/hul/tomato_pick/src/detector/scripts')

import cv2
import time
import open3d as o3d
import copy


from yolov5.detect_tomato import YOLOV5
from yolov5.detect_tomato_classify import YOLOV5Classify
from mmpose.keypoint1 import Keypoints
from utils.startcam import KinectAzure
from utils.startcam import *
from utils.pose import *
from utils.convert_2d_to_3d import *
from utils.distant_pick_list import Determine_pick_list
from rm_75_msgs.msg import *

from rm_75_srvs.srv import *
from std_msgs.msg import *

dir_path = os.path.dirname(os.path.abspath(__file__))
single_weight_path = os.path.join(dir_path + str('/yolov5/weights/distant/best.pt'))


config_file = os.path.join(dir_path + str('/mmpose/configs/tomato/hrnet_w32_h36m_256x256_pck.py'))
checkpoint_file = os.path.join(dir_path + str('/mmpose/tools/weights/hrnet_w32_h36m/best_EPE_epoch_215.pth'))
path_rgb = os.path.join(dir_path + str('test/rgb'))
path_pcd = os.path.join(dir_path + str('test/pcd'))

# class Pick():
#     # def __init__(self):
class Pick():
    def __init__(self):
        rospy.init_node('tomato_vision')
        self.detect = True
        self.move_direction = 0 # move_direction=0 顺着主茎生长方向从左向右
        # self.pre_bbox = [530, 100, 1330, 900]
        self.pre_bbox = [600, 350, 1330, 1000]
        # [530, 200, 1330, 1000] 
        
        
        # 末端can通讯
        # self.bus = can.interface.Bus(bustype = 'slcan', channel='/dev/ttyACM0', bitrate = 500000)
        # self.open = can.Message(arbitration_id=0x141,
        #                             data=[164, 0, 232, 3, 232,110, 3, 0], 
        #                             is_extended_id=False)
        # self.close = can.Message(arbitration_id=0x141,
        #                             data=[164, 0, 144, 1, 0, 0, 0, 0], 
        #                             is_extended_id=False)

        self.detector = YOLOV5Classify(weights=single_weight_path, imgsz=640)  # 初始化目标检测器
        self.keypoint = Keypoints(config_file=config_file, checkpoint_file=checkpoint_file)  # 初始化关键点网络
       
        # 机械臂末端位置
        self.arm_end_state_client = rospy.ServiceProxy("arm_end_state",ArmState)
        # 机械臂移动pose
        self.arm_move_client = rospy.ServiceProxy("move_arm",MoveArm)
        # 机械臂移动joint
        self.arm_move_joint_client = rospy.ServiceProxy("move_arm_joint",MoveArmJoint)
        # # 升降柱移动 
        # self.lift_set_height =rospy.Publisher("/rm_driver/Lift_SetHeight",Lift_Height,queue_size=10)
        
        # # 需通过先向机械臂发布信息
        # # 才能订阅升降状态
        # # 创建定时器，发布获取升降状态的话题"/rm_driver/Lift_GetState"
        # self.lift_get_height = rospy.Publisher("/rm_driver/Lift_GetState",Empty,queue_size=10)
        # self.lift_height = 0
        # self.pub_lift_state = False
        # self.timer = rospy.Timer(rospy.Duration(1),self.timerCallback)

    # def startpick(self):
    def startpick(self, distant_color_image_data, distant_xyz_image, close_color_image_data, close_xyz_image):
        while self.detect:
            # 机械臂到达采摘位姿
            forward_arrive = self.move_joint(type="0")
            # forward_arrive = True
            if forward_arrive:
                # forward_arrive = False
                # 拍摄
                dist_image_shape = distant_color_image_data.shape
                
                # 对远景深度z过滤 0.1-0.8m
                distant_depth = distant_xyz_image[:, 2]
                distant_mask = np.logical_or(distant_depth <= 100, distant_depth >= 800)
                distant_xyz_image[distant_mask, :] = [0, 0, 0]

                # 检测远景目标
                pick_list,distant_image_detect = self.detector.run_distant(distant_color_image_data)
                print(pick_list)
                    
                if len(pick_list):
                    # 2d_3d转换 所有远景串的相机坐标系下近景拍摄点位置 距离30cm
                    locations = converted_distant_2d_3d(pick_list, distant_xyz_image, dist_image_shape)
                    # print(locations)
                    if np.array(locations).size == 0:
                        continue
                        # self.detect =False
                        # break
                else:
                    print("在远景中没有检测到串番茄")
                    continue

                camera_close_positions = []
                camera_close_rotations = [] 
                tomato_classifys = []
                # 将所有远景图像中的目标串转化到空间坐标系内
                for i, location in enumerate(locations):
                    # 针对所有串番茄获得其姿态
                    # print(location[-1])
                    camera_close_rotation_com= Close_Rotation_Single(location[-1])
                    print(location[0: 3])

                    # source = o3d.geometry.PointCloud()
                    # source.points = o3d.utility.Vector3dVector(distant_xyz_image)
                    # coord_frame0 = o3d.geometry.TriangleMesh.create_coordinate_frame(size=100)
                    # coord_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=100)
                    # coord_frame.translate(location[0: 3])
                    # coord_frame.rotate(camera_close_rotation_com)
                    # o3d.visualization.draw_geometries([source, coord_frame, coord_frame0])
        
                    base_close_position, base_close_ori = self.pose_to_base('Arm_Tip', location[0: 3], camera_close_rotation_com)
                    if base_close_position is None:
                        continue
                    camera_close_positions.append(base_close_position)
                    camera_close_rotations.append(base_close_ori)
                    tomato_classifys.append(location[-1])


                   

                for i,camera_close_position in enumerate(camera_close_positions):
                    print("前往第{0}个近景位置进行近景图像采集".format(i))
                    # 根据当前目标与基座在x轴的位置关系 判断是否需要移动升降柱
                    # if camera_close_position[0]
                    print(camera_close_position)
                    tomato_classify = tomato_classifys[i]
                    
                    # ------------------------------------------------------------------------
                    # 机械臂运动到近景拍摄位置
                    camera_arrive = False       
                    camera_arrive = self.move_close(move_link_name='cam_link', 
                                                position=camera_close_positions[i], rotation=camera_close_rotations[i])
                             
                    if camera_arrive == False:
                            print('近景观测位姿不可达')
                            continue
                            # self.detect =False
                            # break
                    else:
                            # 近景深度值过滤
                            close_depth = close_xyz_image[:, 2]
                            close_mask = np.logical_or(close_depth <= 100, close_depth >= 500)
                            close_xyz_image[close_mask, :] = [0, 0, 0]
                            
                            close_image_shape = close_color_image_data.shape
                            # 获取到目标框
                            pre_bbox = {'bbox': self.pre_bbox}
                            pose_results = self.keypoint.pose_detect(close_color_image_data, [pre_bbox])[0]["keypoints"]
                            # pose_results = self.keypoint.pose_detect(close_color_image_data, [tomato_pick[0]])[0]["keypoints"]
                            print("关键点检测结果")
                            print(pose_results)
                        
                            #  相机坐标系下的采摘点和采摘姿态            
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
                                source = o3d.geometry.PointCloud()
                                source.points = o3d.utility.Vector3dVector(close_xyz_image)
                                coord_frame0 = o3d.geometry.TriangleMesh.create_coordinate_frame(size=100)
                                coord_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=100)
                                coord_frame.translate(pick_position)
                                # rotation = tfs.euler.euler2mat(pick_rotation[0],pick_rotation[1],pick_rotation[2],'rxyz')
                                coord_frame.rotate(pick_rotation)
                                o3d.visualization.draw_geometries([source, coord_frame, coord_frame0])
                                
                                # 机械臂运动到采摘位置    
                                # ------------------------------------------------------------------------                        
                                pick = self.move_pick(arm_end_state_link_name='Arm_Tip',move_link_name='tool_link', 
                                                        position=pick_position, rotation=pick_rotation)
                                                        
                                if pick==False:
                                    print("采摘失败")
                                    continue
                                    # self.move_joint(type="0")
                                    # self.detect = False   
                                    # break
                                else:
                                    print("采摘成功")
                                    # self.move_joint(type="1")
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

    
    # 根据升降当前高度和目标高度判断升降的移动
    def move_lift(self,pose,lift):
        # pose为当前机械臂基座和目标之间的高度差 
        # lift为当前升降柱的高度
        new_lift = Lift_Height()
        new_lift.speed = 40
        if 210<=pose<=290:
            return True,pose
        # 升降过高，需要下移
        elif pose<210:
            # 设置合理采摘距离
            Reasonable_height = 220 
            move_distance = Reasonable_height - pose
            # 升降可下降距离为lift 不能过限 lift 最低限制位置为100mm处
            Descent_height_limit = lift -100
            if Descent_height_limit >= move_distance:
                new_lift.height = int(lift - move_distance)
                self.lift_set_height.publish(new_lift)
                # 等待升降执行
                rospy.sleep(2)
                return True, Reasonable_height
            # 超过限制，升降无法到达 
            else:
                print('下降高度受限')
                return False, pose
        # 升降过低，需要上移
        elif pose >290:
             # 设置合理采摘距离
            Reasonable_height = 280 
            move_distance = pose - Reasonable_height
            # 升降可升高距离为900 -lift 不能过限 lift 最高限制位置为900mm处
            Ascent_height_limit = 900 - lift
            if Ascent_height_limit >= move_distance:
                new_lift.height = int(lift + move_distance)
                self.lift_set_height.publish(new_lift)
                # 等待升降执行
                rospy.sleep(2)
                return True, Reasonable_height
            # 超过限制，升降无法到达 
            else:
                print('上升高度受限')
                return False, pose

    def move_close(self, move_link_name, position, rotation):
        # link7  cam_link
        arrive = False
        # rotation rx ry rz
        arrive = self.move_arm(move_link_name,position/1000, rotation,type="0")
        print("机械臂是否到达")
        print(arrive)
        return arrive     
        

    def move_pick(self,arm_end_state_link_name, move_link_name, position, rotation):
        pre_arrive = False
        tool_pick_arrive =False
        back_arrive = False 
        
        # 采摘位置
        base_pick_position, base_pick_ori = self.pose_to_base(arm_end_state_link_name, position, rotation)
        # base_pick_position[2]=base_pick_position[2]-100
        
        # print("目标相对基座坐标系的位置")
        # print(base_pick_position)
        # print(base_pick_ori)
        base_pick_position[1] = base_pick_position[1] - 8

        # 预采摘位置
        base_pre_pick_position = copy.deepcopy(base_pick_position)
        base_pre_pick_position[0] = base_pre_pick_position[0] -100
        
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
                    rospy.sleep(2)
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
    def pose_to_base(self,arm_end_link_name, position_to_cam, rotation_to_cam):
        # --------------------获取机械臂当前位姿-------------
        end_state = self.current_location(arm_end_link_name)
        print(end_state)
        # x y z rx ry rz
        if end_state is not None:
            # 基座坐标
            base_position, base_orientation = cam_to_base(position_to_cam,rotation_to_cam,end_state)
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
        end_state = result_future_end_link
        # x y z rx ry rz
        return end_state
        # else:
        #     return None   

    def move_joint(self,type):
        rospy.wait_for_service("move_arm_joint")
        request_target_joint = MoveArmJointRequest()
        # forward 0/collect 1
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

    # def stop(self):
    #     # 在需要结束线程的地方，设置Event对象，通知线程停止运行
    #     self.stop_event.set()

        # 等待线程结束
        self.th_lift.join()
if __name__ =='__main__':
    try:
        # distant_color_image_data = cv2.imread(r'/home/hul/tomato_pick/sh_close_4.jpg')
        # close_color_image_data = cv2.imread(r'/home/hul/tomato_pick/sh_close_4.jpg')
        # distant_pcd = o3d.io.read_point_cloud(r'/home/hul/tomato_pick/sh_close_4.pcd')
        # close_pcd = o3d.io.read_point_cloud(r'/home/hul/tomato_pick/sh_close_4.pcd')
        # distant_color_image_data = cv2.imread(r'/home/hul/tomato_pick/28.jpg')
        # close_color_image_data = cv2.imread(r'/home/hul/tomato_pick/28.jpg')
        # distant_pcd = o3d.io.read_point_cloud(r'/home/hul/tomato_pick/27.pcd')
        # close_pcd = o3d.io.read_point_cloud(r'/home/hul/tomato_pick/27.pcd')


        # # 
        # # # 获取pcd中xyz数据
        # distant_xyz_image = np.asarray(distant_pcd.points)
        # close_xyz_image = np.asarray(close_pcd.points)

        distant_color_image_data = cv2.imread(r'/home/hul/tomato_pick/20_distant.jpg')
        close_color_image_data = cv2.imread(r'/home/hul/tomato_pick/20_close.jpg')
        distant_xyz_image = np.loadtxt("/home/hul/tomato_pick/20_distant.txt")
        close_xyz_image = np.loadtxt('/home/hul/tomato_pick/20_close.txt')
    

        node = Pick()
        node.startpick(distant_color_image_data, distant_xyz_image, close_color_image_data, close_xyz_image)
        rospy.spin()
        node.stop()
    except rospy.ROSInterruptException:
        pass
    pass

# def main(args=None):

#         rclpy.init(args=args)
#         img0 = cv2.imread(r'/home/hul/PeduncleDetection/mmpose/test/ch_200.jpg')
#         # pcd = o3d.io.read_point_cloud(r'/home/hul/PeduncleDetection/mmpose/test/ch2_30.pcd')
        
#         # # 获取pcd中xyz数据
#         # xyz_image = np.asarray(pcd.points)
#         # depth = xyz_image[:, 2]
#         # mask = np.logical_or(depth <= 200, depth >= 700)
#         # xyz_image[mask, :] = [0, 0, 0]

#         # img0_close = cv2.imread(r'/home/hul/PeduncleDetection/test0/szch1_scenario_8.jpg')
#         # xyz_image_close = np.loadtxt("/home/hul/PeduncleDetection/test0/szch1_scenario_8.txt")
#         # xyz_image_close = xyz_image_close*1000
#         # depth_close = xyz_image_close[:, 2]
#         # mask_close = np.logical_or(depth_close <= 200, depth_close >= 350)
#         # xyz_image_close[mask_close, :] = [0, 0, 0]

#         Pick_node = Pick("tomato_vision")
#         # Pick_node.startpick(img0, xyz_image)
#         # Pick_node.startpick(img0, xyz_image,img0_close, xyz_image_close)
#         Pick_node.startpick()
#         rclpy.spin(Pick_node)
#         Pick_node.destroy_node()
#         rclpy.shutdown()

# if __name__ == "__main__":
#     # image_path = 'E:/Peduncle_Detection/mmpose/test/107.jpg'

#     img0 = cv2.imread(r'/home/hul/PeduncleDetection/mmpose/test/ch2_4.jpg')
#     pcd = o3d.io.read_point_cloud(r'/home/hul/PeduncleDetection/mmpose/test/ch2_4.pcd')
#     # 获取pcd中xyz数据
#     xyz_image = np.asarray(pcd.points)
#     # 获取pcd中z数据 深度
#     # cv2.imshow('img', img0)
#     # cv2.waitKey(5)
#     dete = Pick()
#     dete.startpick(img0, xyz_image)
#     pass
