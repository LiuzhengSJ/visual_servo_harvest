#! /home/liu/anaconda3/envs/mmpose1/bin/python
# -*- coding: utf-8 -*-

import rospy
from yolov5.detect_tomato_classify import YOLOV5Classify
from mmpose.keypoint1 import Keypoints
from cv_bridge import CvBridge
import cv2
from rm_75_srvs.msg import *
from rm_75_srvs.srv import *

class DetectionNode:
    def __init__(self): 
        rospy.init_node('detection_node', anonymous=True)
        
        # 订阅来自相机的图像 DetectObjects是远景检测自定义服务类型
        self.distant_service = rospy.Service('/detect_objects', DetectObjects, self.distant_detect)
        rospy.loginfo("目标检测服务启动")
        
        self.keypoint_service = rospy.Service('/detect_keypoints', DetectKeypoints, self.keypoint_detect)
        rospy.loginfo("关键点检测服务启动")
    

        # 初始化目标检测和关键点检测
        self.distant_detector = YOLOV5Classify(weights="/home/liu/realman/src/detector/scripts/yolov5/weights/distant/best.pt", imgsz=640)
        self.keypoint_detector = Keypoints(config_file="/home/liu/realman/src/detector/scripts/mmpose/configs/tomato/hrnet_w32_h36m_256x256_pck.py",
                                  checkpoint_file="/home/liu/realman/src/detector/scripts/mmpose/tools/weights/hrnet_w32_h36m/best_EPE_epoch_215.pth")

        self.bridge = CvBridge()
        self.pre_bbox = [400, 200, 800, 800]
    def distant_detect(self,req):
        cv_image = self.bridge.imgmsg_to_cv2(req.image, "bgr8")
        
        detected_objects,_=self.distant_detector.run_distant(cv_image)
        #run_distant会返回一个检测后的效果图，后面可以保存下来
        
        response = DetectObjectsResponse()
        for obj in detected_objects:
            bbox = BoundingBox(xmin=obj[0], ymin=obj[1], xmax=obj[2], ymax=obj[3],cls=int(obj[4]))
            response.boxes.append(bbox)
        return response

    def keypoint_detect(self,req):
        cv_image = self.bridge.imgmsg_to_cv2(req.image,"bgr8")
        rospy.loginfo(f"Image shape: {cv_image.shape}, dtype: {cv_image.dtype}")
        # 运行关键点检测
        keypoints_list = self.keypoint_detector.pose_detect(cv_image)
        print("关键点检测完成")
        keypoints = keypoints_list[0]["keypoints"]
        # 组织响应
        response = DetectKeypointsResponse()
        for kp in keypoints:
            response.keypoints.append(Keypoint(x=kp[0], y=kp[1], confidence=kp[2]))

        return response

if __name__ == "__main__":
    DetectionNode()
    
    rospy.spin()