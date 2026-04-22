#! /home/liu/anaconda3/envs/mmpose1/bin/python
# -*- coding: utf-8 -*-

import numpy as np

# from utils.new_convert_2d_to_3d import *
from scipy.spatial.transform import Rotation as R
import transforms3d as tfs
# import open3d as o3d
from math import pi
def convert_pose_to_matrix(pose,type="euler"):
#输入位移pose转换为对应的齐次矩阵
#pose=xyzrxryrz 欧拉角
# #法奥机械臂为毫米制，睿尔曼米制
    x = pose[0]
    y = pose[1]
    z = pose[2]
    T = np.array([x,y,z])
    if type=="euler":
        if len(pose)!=6:
            raise ValueError("Euler pose requires 6 elements:xyzrxryrz")
        rx = pose[3]*pi/180
        ry = pose[4]*pi/180
        rz = pose[5]*pi/180
        # rx = pose[3]
        # ry = pose[4]
        # rz = pose[5]
        if rx<3 and ry<3 and rz<3:
            print("输入的欧拉角可能已经为弧度值，注意检查")
        rotation = tfs.euler.euler2mat(rx,ry,rz,'sxyz')
    #pose为四元数
    elif type=="wxyz":
        if len(pose)!=7:
            raise ValueError("Quaternion pose requires 7 elements: txtytzwxwywz")
        
        w = pose[3]
        qx = pose[4]
        qy = pose[5]
        qz = pose[6]
        rotation = tfs.quaternions.quat2mat([w,qx,qy,qz])
    else:
        print("pose类型错误")
    temp = np.hstack((rotation,T.reshape(3,1)))
    homogeneous_matrix =np.vstack((temp,[0,0,0,1]))
    #返回齐次矩阵与旋转矩阵
    return homogeneous_matrix,rotation       
def target_pose_to_base(target_pose,cam_rotation,endstate):
    '''
    target_pose:    target position in distant camera frame, m
    cam_rotation:   the specific orientation to closely observe the tomatoes. Constant according to the specific tomato type. in distant camera frame
    endstate:       pose of end-effector, in robot base frame
    Return
    base_object_pose    close camera position in base frame
    euler_cam_base      close camera euler pose in base frame
    '''
    #相机标定
    #法奥为毫米制，欧拉角为角度制！！！
    # r_end_to_cam = np.array([[-0.99936258,0.03187667,0.01607177],
    #                             [-0.03098483,-0.99811471,0.05298081],
    #                             [0.01773032,0.05244906,0.99846619]])
    #   # shape=(3, 3)
    r_end_to_cam = np.array([[0.99936258,-0.0001920,-0.0002967],
                                [0.0001917,0.999999,-0.0009425],
                                [0.0002968,0.0009424,0.99846619]]) 
    # t_cam_to_end = np.array([74.139,2.225,74.221])
    t_end_to_cam = np.array([-0.01022937,-0.11429086,0.08172984])
    temp = np.hstack((r_end_to_cam,t_end_to_cam.reshape(3,1)))
    end_cam_mat =np.vstack((temp,[0,0,0,1]))
    base_end_mat,base_end_rotation = convert_pose_to_matrix(endstate)
    print("末端到基座的齐次矩阵为")
    print(base_end_mat)

    cam_base_mat = np.dot(base_end_mat,end_cam_mat)

    print("相机到基座的齐次矩阵为")
    print(cam_base_mat)
    target_pose_m = np.empty(4)
    target_pose_m[0]=target_pose[0]
    target_pose_m[1]=target_pose[1]
    target_pose_m[2]=target_pose[2]
    target_pose_m[3]=1
    print("相机坐标系下的目标点位置为")
    print(target_pose_m)


    base_object_pose_m = np.dot(cam_base_mat,target_pose_m)
    base_object_pose = base_object_pose_m[:3]
    print('基坐标系下的目标位置为')
    print(base_object_pose)
    #法奥机械臂sdk返回和要求输入都是毫米制！！
    # base_object_pose[0]=base_object_pose[0]*1000
    # base_object_pose[1]=base_object_pose[1]*1000
    # base_object_pose[2]=base_object_pose[2]*1000
    
    rotation_cam_base = np.dot(r_end_to_cam,cam_rotation)
    rotation = np.dot(base_end_rotation,rotation_cam_base)

    #   pose, camera in base
    euler_cam_base = tfs.euler.mat2euler(np.asarray(rotation),'sxyz')

    print("基坐标系下的旋转姿态为")
    print(euler_cam_base)
    # euler_cam_base_list = list(euler_cam_base)
    # euler_cam_base_list[0]=-109.827
    # euler_cam_base_list[1]=5.483
    # euler_cam_base_list[2]=-81.238
    # euler_cam_base = tuple(euler_cam_base_list)
    return base_object_pose,euler_cam_base

def pick_cam_ori(keypoints_list,keypoints_index,tomato_classify):
    '''
    keypoints_list:    list of keypoints 3D positions
    keypoints_index:    list of keypoints validation status, 1 or 0
    '''
 
    # ----------------------model-------------------------
    # 获取果梗上关键点非零值的索引 sd n su
    stem_indexs = np.nonzero(keypoints_index[0:3])[0]

    stem_keypoints = []
    stem_depths = []
    
    for stem_index in stem_indexs:
        stem_keypoints.append(keypoints_list[stem_index])
        stem_depths.append(keypoints_list[stem_index][2])
    
    # 获取果上关键点非零值的索引 peduncle n m d
    stalk_indexs = np.nonzero(keypoints_index[3:5])[0]
    stalk_keypoints = []
    stalk_ys = []
    if keypoints_index[1] == 1:
         stalk_keypoints.append(keypoints_list[1])
         stalk_ys.append(keypoints_list[1][1])
    for stalk_index in stalk_indexs:
        stalk_keypoints.append(keypoints_list[3:5][stalk_index])
        stalk_ys.append(keypoints_list[3:5][stalk_index][1])
    
    # 获取果上关键点非零值的索引 peduncle l r 
    fruit_indexs = np.nonzero(keypoints_index[5:7])[0]
    fruit_keypoints = []
    for fruit_index in fruit_indexs:
        fruit_keypoints.append(keypoints_list[5:7][fruit_index])

    # 计算旋转角度
    Rota_T =[]
    if len(stalk_keypoints) >= 2 and len(stem_keypoints) >= 2:
        if stem_keypoints[0][0] <= stalk_keypoints[-1][0]:
            tendency = "0"  # 果梗右倾
        else:
            tendency = "1"  # 果梗左倾

        stem_vector = stem_keypoints[1] - stem_keypoints[0]  # [3,]

        stalk_vector = stalk_keypoints[-1] - stalk_keypoints[0]
        if len(fruit_keypoints)>=1:
            # fruit_vector = fruit_keypoints[0] - stem_keypoints[-1]
            fruit_vector = fruit_keypoints[0] - stem_keypoints[1]
        else:
            fruit_vector = stalk_keypoints[-1] - stem_keypoints[1]
        if tendency == "0":
            z = np.cross(stem_vector, stalk_vector)

            x = np.cross(fruit_vector, z)# 以茎和果梗平面的法向量为 工具坐标z轴方向 垂直向下里
        else:
            z = -np.cross(stem_vector, stalk_vector)
            x = np.cross(fruit_vector, z)
        # 以Z轴和主茎平面的法向量为 工具坐标Y轴方向 指向
        y = np.cross(z, x)  # 以y轴和z轴为 工具坐标Y轴方向 指向右侧
        # 旋转矩阵
        Rota_T.append(x/np.linalg.norm(x))
        Rota_T.append(y/np.linalg.norm(y))
        Rota_T.append(z/np.linalg.norm(z))
        Rota = np.transpose(Rota_T)
        # print(Rota)
        rota_angle = tfs.euler.mat2euler(Rota,'rxyz')
        print(f"计算所的旋转角度：{rota_angle}")
        rx = rota_angle[0]
        ry = rota_angle[1]
        rz = rota_angle[2]

#   limit the range of rotations to a small values, HOW TO INTERPRET?
        if tomato_classify == 1.0:
            # 在x轴上，即俯仰角度 
            # 近景观测时俯视10度 -10 0.174533
            # -8度  -5度
            if -0.139626 <= rota_angle[0] <= -0.087267:
                rx = rota_angle[0]
            elif rota_angle[0]>= -0.087267:
                rx = -0.087267
            elif rota_angle[0]<=-0.139626:
                rx = -0.139626

            # 在y轴上，即摇摆角度 
            # 近景观测时0度 
            # -10度 10度
            if -0.174533 <= rota_angle[1] <= 0.174533:
                ry = rota_angle[1]
            elif rota_angle[1]>= 0.174533:
                ry = 0.174533
            elif rota_angle[0]<=-0.174533:
                ry = -0.174533

            # -20度 0.349066
            if -0.349066 <= rota_angle[2] <= 0.349066:
                rz = rota_angle[2]
            elif rota_angle[2]>= 0.349066:
                rz = 0.349066
            elif rota_angle[2]<=-0.349066:
                rz = -0.349066
        else:
            # 在x轴上，即俯仰角度 
            # 近景观测时俯视10度 -10 0.174533
            # -5度  15度
            if -0.087267 <= rota_angle[0] <= 0.2618:
                rx = rota_angle[0]
            elif rota_angle[0]>= 0.2618:
                rx = 0.2618
            elif rota_angle[0]<=-0.087267:
                rx = -0.087267

            # 在y轴上，即摇摆角度 
            # 近景观测时-5  5度 
            # -15   15 
            if -0.2618 <= rota_angle[1] <= 0.2618:
                ry = rota_angle[1]
            elif rota_angle[1]>= 0.2618:
                ry = 0.2618
            elif rota_angle[1]<=-0.2618:
                ry = -0.2618

            # -80度 
            if -1.396264 <= rota_angle[2] <= 1.396264:
                rz = rota_angle[2]
            elif rota_angle[2]>= 1.396264:
                rz = 1.396264
            elif rota_angle[2]<=-1.396264:
                rz = -1.396264  
        
        Rota = tfs.euler.euler2mat(rx,ry,rz,'rxyz')
        
        # Rota = []
        # Rota.append(rx)
        # Rota.append(ry)
        # Rota.append(rz)
        
    else:
        return np.asarray([False]), np.asarray([False])
        # return None, None

    # sd n su m d l r
    #  pick_pose m点 或者 n、d中点
    # 主茎上确定起码有两点 sd0 n1 su2
    # 果梗上起码有一点 m3 d4
    # sd n su m d l r
    # "0": "back",
    # "1": "forward",
    # "2": "left",
    # "3": "right" 
    
    if keypoints_index[3] != 0:
        pick_pose = keypoints_list[3]
    else:  # m若不存在  d4肯定存在
        if keypoints_index[1] != 0:

            pick_pose = keypoints_list[4] - (keypoints_list[4] - keypoints_list[1])/2

        else:  # n若不存在 sd0 su2肯定同时存在
            n = (keypoints_list[0] + keypoints_list[2])/2

            pick_pose = n - (keypoints_list[4] - n) / 2

    
    if tomato_classify == 1:
     
        stem_depth=np.min(stem_depths)
        if pick_pose[2] >= (stem_depth + 0.01):
            pick_pose[2] = stem_depth - 0.04
        else:
            # pick_pose[2] = pick_pose[2] - 10
            pick_pose[2] = pick_pose[2] - 0.03
        stalk_y = np.median(stalk_ys)

        if pick_pose[1] <= stalk_y - 0.05:
            pick_pose[1] =stalk_y + 0.01
        else:
            pick_pose[1] = pick_pose[1] + 0.01



        # ------------------------------------
    return pick_pose, np.array(Rota)
if __name__ == "__main__":
    rotation = [[1,0,0],
                [0,1,0],
                [0,0,1]]
    r_end_to_cam = np.array([[-0.0448469,-0.9914312,-0.1226901],
                            [0.9988587,-0.0424811,-0.0218320],
                            [0.0164329,-0.1235292,0.9922048]])  # shape=(3, 3)
    # t_cam_to_end = np.array([74.139,2.225,74.221])
    t_end_to_cam = np.array([0.074139,0.002225,0.074221])
    # end_cam_mat = convert_to_homogeneous_matrix(cam_end_pose)
    temp = np.hstack((r_end_to_cam,t_end_to_cam.reshape(3,1)))
    end_cam_mat =np.vstack((temp,[0,0,0,1]))
    r_base_end = np.array([[0.99807,0.033948,0.051977],
                            [-0.055387,0.10875,0.99252],
                            [0.028041,-0.99349,0.11042]])
    rotation_1 = np.dot(r_base_end,r_end_to_cam)
    print(rotation_1)
    rotation_11 = tfs.euler.mat2euler(rotation_1,'sxyz')
    # base_object_pose = np.dot(base_cam_mat,pose)
    # end_object_pose = np.dot(cam_end_mat,pose)
    # base_object_pose = np.dot(end_base_mat,end_object_pose)
    print(rotation_11)