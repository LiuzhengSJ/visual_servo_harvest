#! /home/hul/anaconda3/envs/torch/bin/python3
# -*- coding: utf-8 -*-

# 将番茄矩形框的中点从图像坐标系转到相机坐标系
# ['left',"forward","right","back"]
import math
# from aubo.cam_to_base import convert_cam_to_base
# from aubo.robotcontrol import *
import numpy
import numpy as np
from scipy.spatial.transform import Rotation as R
# [right forward back left]

def converted_distant_2d_3d_demostration(camera_close_list, xyz_distant_image, image_shape):
    locations_3d = []
    # 获取pcd中xyz数据
    xyz_close_point = np.asarray(xyz_distant_image).reshape(image_shape[0], image_shape[1], 3)
    # 获取pcd中z数据 深度
    xyz_distant_depth_z = xyz_close_point[:, :, -1]
    xyz_distant_depth_xy = xyz_close_point[:, :, 0:2]

    for list in camera_close_list:
        # 目标框内的深度值z
        crop_distant_depths = xyz_distant_depth_z[int(list[1]):int(list[3]), int(list[0]):int(list[2])]  # y在前 x在后
      
        # 滤除深度信息为0 
        # 若滤除后深度信息全为0，目标框内无番茄
        crop_distant_depths_filtered = crop_distant_depths[crop_distant_depths[:,:]!=0]
        if crop_distant_depths_filtered.size==0:
            continue

        crop_distant_median_depths = np.median(crop_distant_depths_filtered)

        # 拍摄点z值距离0.35m
        crop_distant_median_cam_depths = crop_distant_median_depths-300

        # 拍摄点
        # 目标框内的xy
        xy_3d = []
        xy_3d_crop_all = xyz_distant_depth_xy[int(list[1]):int(list[3]), int(list[0]):int(list[2]), :]
        xy_3d_crop_all = xy_3d_crop_all.reshape(-1, 2)
        # 滤除xy信息为0 
        xy_3d_crop_all_filtered = xy_3d_crop_all[np.any(xy_3d_crop_all != [0, 0], axis=1)]
        if xy_3d_crop_all_filtered.size ==0:
            continue

        # 分别获取x像素和y像素
        x = xy_3d_crop_all_filtered[:,0]
        
        # 取x像素中间值
        x_c = np.median(x)

        # y取最大值
        y = xy_3d_crop_all_filtered[:,1]
        y_c = y[0] -20
        # y_c = y[0] -50

        xy_3d = np.append(xy_3d, x_c)
        xy_3d = np.append(xy_3d, y_c)
        # 添加深度值z
        xy_3d = np.append(xy_3d, crop_distant_median_cam_depths)
        # print(xy_3d)
        locations_3d.append(xy_3d)
    return locations_3d

def converted_distant_2d_3d(camera_close_list, xyz_distant_image, image_shape):
    '''
    input:
    camera_close_list:  list of prediction boxes, each values contains ： xmin, ymin, xmax, ymax, class
    xyz_distant_image:  point cloud data, xyz, unit: m
    image_shape:        image shape, 1280X720X3？

    return:
    close observation location: x:  the median value of all validated values.
                                y:  the top (first) element value along image y axis.
                                z:   the median value of all validated values - 0.25 m
                                class:  the tomato class, 0,1,2,3
    '''
    locations_3d = []
    # 获取pcd中xyz数据
    xyz_close_point = np.asarray(xyz_distant_image).reshape(image_shape[0], image_shape[1], 3)
    # 获取pcd中z数据 深度
    xyz_distant_depth_z = xyz_close_point[:, :, -1]
    xyz_distant_depth_xy = xyz_close_point[:, :, 0:2]
    
    for list in camera_close_list:
        x1 = int(list['xmin'])
        y1 = int(list['ymin'])
        x2 = int(list['xmax'])
        y2 = int(list['ymax'])
        class_id = int(list['class'])
        # 目标框内的深度值z
        crop_distant_depths = xyz_distant_depth_z[y1:y2, x1:x2]  # y在前 x在后
      
        # 滤除深度信息为0 
        # 若滤除后深度信息全为0，目标框内无番茄
        crop_distant_depths_filtered = crop_distant_depths[crop_distant_depths[:,:]!=0]
        if crop_distant_depths_filtered.size==0:
            continue
        
        crop_distant_median_depths = np.median(crop_distant_depths_filtered)
        # 拍摄位置的z值距后退
        crop_distant_median_cam_depths = crop_distant_median_depths-0.25

        # 拍摄点
        # 目标框内的xy
        xy_3d = []
        xy_3d_crop_all = xyz_distant_depth_xy[y1:y2, x1:x2, :]
        xy_3d_crop_all = xy_3d_crop_all.reshape(-1, 2)
        # 滤除xy信息为0 
        xy_3d_crop_all_filtered = xy_3d_crop_all[np.any(xy_3d_crop_all != [0, 0], axis=1)]
        if xy_3d_crop_all_filtered.size ==0:
            continue

        # 获取x像素像素
        x = xy_3d_crop_all_filtered[:,0]
        
        # 取x像素中间值
        x_c = np.median(x)

        # 获取y像素
        y = xy_3d_crop_all_filtered[:,1]
        # 取y像素的最大值
        y_c = y[0]
        # y_c = y[0] -50

        xy_3d = np.append(xy_3d, x_c)
        xy_3d = np.append(xy_3d, y_c)
        
        # 添加深度值z
        xy_3d = np.append(xy_3d, crop_distant_median_cam_depths)
        # 添加目标class
        xy_3d = np.append(xy_3d, class_id)  
        locations_3d.append(xy_3d)
    return locations_3d


def converted_keypoint_2d_3d(pose_results, xyz_close_image, image_shape):
    '''
    input:
    pose_results:       list of dicts indicating key points 'x', 'y' pixel position, and 'conf'
    xyz_close_image:    point cloud
    image_shape:
    output:
    keypoints_3d:       locations of key points, x, y, z in camera frame, unit: m
    keypoints_label:    1 for valid, 0 or invalid
    '''
    keypoints_3d = []
    keypoints_label = []
    # 获取pcd中xyz数据
    xyz_close_point = np.asarray(xyz_close_image).reshape(image_shape[0], image_shape[1], 3)
    # 获取pcd中z数据 深度
    xyz_close_depth_z = xyz_close_point[:, :, -1]
    xyz_close_depth_xy = xyz_close_point[:, :, 0:2]

    # 调换su和n的位置
    pose_results[1],pose_results[2] = pose_results[2],pose_results[1]

    for list in pose_results:
        # 深度值
        # 关键点检测score
        if list['confidence'] <= 0.2:
            keypoints_label.append(0)
            keypoints_3d.append(np.array([0, 0, 0]))
            continue
        crop_close_depths = xyz_close_depth_z[int(list['y']-5):int(list['y']+5), int(list['x']-5):int(list['x']+5)]
        crop_close_depths_filtered = crop_close_depths[crop_close_depths[:, :] != 0]
        crop_close_median_depths = np.median(crop_close_depths_filtered)
        # if 0.2 <= crop_close_median_depths <= 0.5:  # m
        if 0.2 <= crop_close_median_depths <= 0.8:  # mm
            keypoint_crop_3d = xyz_close_depth_xy[int(list['y']-5):int(list['y']+5), int(list['x']-5):int(list['x']+5), :]
            keypoint_crop_3d = keypoint_crop_3d.reshape(-1, 2)
            keypoint_crop_3d_filtered = keypoint_crop_3d[np.any(keypoint_crop_3d != [0, 0], axis=1)]
            keypoint_3d = np.median(keypoint_crop_3d_filtered, axis=0)
            keypoint_3d = np.append(keypoint_3d, crop_close_median_depths)
            label = 1
        else:
            print("出现深度缺失问题")
            keypoint_3d = xyz_close_depth_xy[int(list['y']), int(list['x']), :]  # y在前 x在后
            keypoint_3d = np.append(keypoint_3d, 0)
            label = 0
        keypoints_label.append(label)
        keypoints_3d.append(keypoint_3d)
    return keypoints_3d, keypoints_label

# x
# [1, 0, 0],
# [0, np.cos(), -np.sin()]
# [0, np.sin(), np.cos()]
# y
# [np.cos(), 0, np.sin()]
# [0, 1, 0],
# [-np.sin(), 0, np.cos()]
# z
# [np.cos(), -np.sin(), 0]
# [np.sin(), np.cos(), 0]
# [0, 0, 1]



# # "0": "back",
# # "1": "forward",
# # "2": "left",
# # "3": "right" 
def Close_Rotation_Single(cls):
    if cls == 2:  # left
        # 绕相机坐标系y轴旋转
        Rotation_matrix_y = np.array([[np.cos(-np.pi / 36), 0, np.sin(-np.pi / 36)],
                                    [0, 1, 0],
                                    [-np.sin(-np.pi / 36), 0, np.cos(-np.pi / 36)]])
        # 绕相机坐标系x轴旋转-俯视
        Rotation_matrix_x = np.array([[1, 0, 0],
                                    [0, np.cos(-np.pi / 18), -np.sin(-np.pi / 18)],
                                    [0, np.sin(-np.pi / 18), np.cos(-np.pi / 18)]])
        
        Rotation_matrix = np.dot(Rotation_matrix_y,Rotation_matrix_x)
    elif cls == 1 or cls ==0:  # forward
        # 绕相机坐标系x轴旋转
        Rotation_matrix = np.array([[1, 0, 0],
                                    [0, np.cos(-np.pi / 18), -np.sin(-np.pi / 18)],
                                    [0, np.sin(-np.pi / 18), np.cos(-np.pi / 18)]])
    elif cls == 3:  # right
        # 绕相机坐标系y轴旋转
        Rotation_matrix_y = np.array([[np.cos(np.pi / 36), 0, np.sin(np.pi / 36)],
                                    [0, 1, 0],
                                    [-np.sin(np.pi / 36), 0, np.cos(np.pi / 36)]])
        # 绕相机坐标系x轴旋转-俯视
        Rotation_matrix_x = np.array([[1, 0, 0],
                                    [0, np.cos(-np.pi / 18), -np.sin(-np.pi / 18)],
                                    [0, np.sin(-np.pi / 18), np.cos(-np.pi / 18)]])
        
        Rotation_matrix = np.dot(Rotation_matrix_y,Rotation_matrix_x)
        # Rotation_matrix = Rotation_matrix_x
    return Rotation_matrix



# #  0 right 1 forward 2 back 3 left
def Close_Rotation(cls):
    cls = int(cls)
    print(cls)
    if cls == 3:  # left  

        # 绕相机坐标系y轴旋转
        Rotation_matrix_y = np.array([[np.cos(np.pi / 18), 0, np.sin(np.pi / 18)],
                                    [0, 1, 0],
                                    [-np.sin(np.pi / 18), 0, np.cos(np.pi / 18)]])
        # 绕相机坐标系x轴旋转-俯视
        Rotation_matrix_x = np.array([[1, 0, 0],
                                    [0, np.cos(np.pi / 12), np.sin(np.pi / 12)],
                                    [0, -np.sin(np.pi / 12), np.cos(np.pi / 12)]])
        
        Rotation_matrix = np.dot(Rotation_matrix_y,Rotation_matrix_x)
        # Rotation_matrix = Rotation_matrix0        
                                              
    elif cls == 1:  # forward
        # 绕相机坐标系x轴旋转
        Rotation_matrix = np.array([[1, 0, 0],
                                    [0, np.cos(np.pi / 12), np.sin(np.pi / 12)],
                                    [0, -np.sin(np.pi / 12), np.cos(np.pi / 12)]])
    elif cls == 0:  # right
         # 绕相机坐标系y轴旋转
        Rotation_matrix_y = np.array([[np.cos(-np.pi / 18), 0, np.sin(-np.pi / 18)],
                                    [0, 1, 0],
                                    [-np.sin(-np.pi / 18), 0, np.cos(-np.pi / 18)]])
        # 绕相机坐标系x轴旋转-俯视
        Rotation_matrix_x = np.array([[1, 0, 0],
                                    [0, np.cos(np.pi / 12), np.sin(np.pi / 12)],
                                    [0, -np.sin(np.pi / 12), np.cos(np.pi / 12)]])
        
        Rotation_matrix = np.dot(Rotation_matrix_y,Rotation_matrix_x)

    return Rotation_matrix


def Demo_Close_Rotation():
        # 绕相机坐标系x轴旋转
    Rotation_matrix = np.array([[1, 0, 0],
                                    [0, np.cos(np.pi / 18), np.sin(np.pi / 18)],
                                    [0, -np.sin(np.pi / 18), np.cos(np.pi / 18)]])
   
    return Rotation_matrix



















