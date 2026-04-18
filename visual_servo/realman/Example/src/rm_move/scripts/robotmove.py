#! /home/liu/anaconda3/envs/mmpose1/bin/python
# -*- coding: utf-8 -*-

import transforms3d as tfs
import numpy as np
from numpy import float64
from robotic_arm_package.robotic_arm import *
import sys
import sys
# print(sys.path)
import rospy
from rm_75_srvs.srv import *
from rm_75_srvs.srv import MoveArmJoint,MoveArmJointResponse,\
    MoveArm,MoveArmResponse,ArmState,ArmStateResponse
class robot:
    def __init__(self):
        rospy.init_node('robot_move')
        self.robot = Arm(RM75, "192.168.1.18")
         # 运动到rm_75.srdf中预设的位姿
        self.joint_move_sersvice = rospy.Service('move_arm_joint',MoveArmJoint,self.JointMove)
        # 按照视觉节点给定位姿进行路径规划并运动
        self.arm_move_service = rospy.Service('move_arm',MoveArm,self.ArmMove)
        # 获取目标关节相对于base_link的位姿
        self.arm_state_service = rospy.Service('arm_end_state',ArmState,self.ArmEndState)
        # 初始姿态
        # self.forward = [55, -90, -18, 110, -58, 70, 0]
        self.forward = [0,-35.088,-33.5,61.88,105.728,115.56296,-111.531]
        # 收集姿态1、2
        self.collect1 = [55, -90, -18, 110, -58, 70, 0]
        self.collect2 = [55, -90, -18, 110, -58, 70, 0]
        # [18.44, -10.677, -124.158, -15, -71.455]
      
    
    # 预设的位姿
    def JointMove(self,req):
        joint_res = MoveArmJointResponse()
        # 初始姿态 远景姿态
        if req.move_point.strip() == '0':
            # Movej_Cmd(self, joint, v, r=0, block=True):
            ret=self.robot.Movej_Cmd(self.forward,5)
            if ret == 0:
                print("初始位置成功:" + str(ret))
                joint_res.arrive = True
                return joint_res
            else:
                print("初始位置失败:" + str(ret))
                joint_res.arrive = False
                return joint_res
        # 收集姿态
        elif req.move_point.strip() == '1':
            collect_ret1=self.robot.Movej_Cmd(self.collect1,5)
            if collect_ret1 == 0:
                collect_ret2=self.robot.Movej_Cmd(self.collect2,5)
                if collect_ret2 ==0:
                    print("收集位置2成功："+str(collect_ret2))
                    joint_res.arrive = True
                    return joint_res
                else:
                    print("收集位置2失败："+str(collect_ret2))
                    joint_res.arrive = False
                    return joint_res
            else:
                print("收集位置1失败："+str(collect_ret1))
                joint_res.arrive = False
                return joint_res
        # 收集姿态
        elif req.move_point.strip() == '2':
            collect_ret3=self.robot.Movej_Cmd(self.collect1,5)
            if collect_ret3 ==0:
                print("收集位置1成功："+str(collect_ret3))
                joint_res.arrive = True
                return joint_res
            else:
                print("收集位置1失败："+str(collect_ret3))
                joint_res.arrive = False
                return joint_res  

    # 给定位姿进行路径规划
    def ArmMove(self,req):
        arm_res = MoveArmResponse()
        # 选择机械臂的规划方式
        if req.move_type.strip() == '0':
            # 关节空间下的运动规划 
            # Movej_P_Cmd(self, pose, v, r=0, block=True):关节空间运动到目标位姿 
            move_link = req.target_pose.header.frame_id
            # 改变规划的工具坐标
            chang_tool_ret = self.robot.Change_Tool_Frame(move_link,False)
            rospy.sleep(0.5)
            # pose是x y z rx ry rz
            # req 是 x y z w x y z
            # 不想重新自定义服务了，凑合用吧
            #这里是将四元数表示的姿态转为欧拉角的姿态？liuy
            pose=[]
            pose.append(req.target_pose.pose.position.x)
            pose.append(req.target_pose.pose.position.y)
            pose.append(req.target_pose.pose.position.z)
            pose.append(req.target_pose.pose.orientation.x)
            pose.append(req.target_pose.pose.orientation.y)
            pose.append(req.target_pose.pose.orientation.z)
            #移动到目标姿态
            movej_P_ret = self.robot.Movej_P_Cmd(pose,5)
            if movej_P_ret==0:
                print(str(move_link)+'在关节空间运动到指定目标位姿')
                arm_res.arrive = True
                return arm_res
            else:
                print(str(move_link)+'在关节空间未到达指定目标位姿')
                arm_res.arrive = False
                return arm_res
        elif req.move_type.strip() == '1':
            # 笛卡尔空间直线运动
            # Movel_Cmd(self, pose, v, r=0, block=True):
            arm_res = MoveArmResponse()
            move_link = req.target_pose.header.frame_id
            chang_tool_ret = self.robot.Change_Tool_Frame(move_link,False)
            rospy.sleep(0.5)
            pose=[]
            pose.append(req.target_pose.pose.position.x)
            pose.append(req.target_pose.pose.position.y)
            pose.append(req.target_pose.pose.position.z)
            pose.append(req.target_pose.pose.orientation.x)
            pose.append(req.target_pose.pose.orientation.y)
            pose.append(req.target_pose.pose.orientation.z)
            Movel_ret = self.robot.Movel_Cmd(pose,5)
            if Movel_ret == 0:
                print(str(move_link)+'在笛卡尔空间运动到指定目标位姿')
                arm_res.arrive = True
                return arm_res
            else:
                print(str(move_link)+'在笛卡尔空间未到达指定目标位姿')
                arm_res.arrive = False
                return arm_res
            
    # 目标关节相对于base_link的位姿
    def ArmEndState(self,req):
        endstate_res = ArmStateResponse()
        end_link = req.link_name
        chang_tool_ret = self.robot.Change_Tool_Frame(end_link,False)
        rospy.sleep(0.5)
        ret, joint, pose, arm_err, sys_err = self.robot.Get_Current_Arm_State(retry=1)
        endstate_res.link_state.orientation.w = 0
        endstate_res.link_state.orientation.x = pose[3]
        endstate_res.link_state.orientation.y = pose[4]
        endstate_res.link_state.orientation.z = pose[5]
        endstate_res.link_state.position.x = pose[0]
        endstate_res.link_state.position.y = pose[1]
        endstate_res.link_state.position.z = pose[2]
        return endstate_res

if __name__ == '__main__':
    try:
        robot()
        rospy.spin()
    except rospy.ROSInterruptException:
        print('shibai')