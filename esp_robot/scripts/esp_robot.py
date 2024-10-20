#!/usr/bin/python3
# coding=UTF-8

import cv2
import numpy as np
import rospy
import yaml
import time
import sys
import math
import string
import os
import threading
import rospkg
import tf
import copy
from socket import *

from std_msgs.msg import Header
from sensor_msgs.msg import Image
from cv_bridge import CvBridge , CvBridgeError
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

class ESP_ROBOT:
    def __init__(self):
        self.esp_url = str(rospy.get_param('esp_url','http://192.168.43.37'))
        self.esp_ctrl_url = str(rospy.get_param('ctrl_url','192.168.43.37'))
        self.esp_control_port = int(rospy.get_param('ctrl_port',81))
        self.camera_image_topic = rospy.get_param('/camera_image_topic','esp_cam/image_raw') #图像话题名字
        self.camera_info_topic = rospy.get_param('/camera_info_topic','esp_cam/camera_info') #摄像头信息名字

        self.imu_topic = rospy.get_param('/imu_topic','imu')  #imu数据
        self.imu_frame_id = rospy.get_param('/imu_frame_id','imu_link') #imulink名字

        self.odom_topic = rospy.get_param('/odom_topic','odom') #odom话题名字
        self.odom_tf_switch = bool(rospy.get_param('/odom_tf_switch',True)) #odom tf变换 是否发布
        self.odom_frame_id = rospy.get_param('/odom_frame_id','base_link') #odom link名字
        
        self.cmd_topic = rospy.get_param('/cmd_topic','cmd_vel') #控制话题名字

        # self.chassis_led_r = int(rospy.get_param('/led_r',255)) #red 0-255 LED颜色设置
        # self.chassis_led_g = int(rospy.get_param('/led_g',0)) #green
        # self.chassis_led_b = int(rospy.get_param('/led_b',0)) #blue

        # self.chassis_shape_a = float(rospy.get_param('/chassis_shape_a',0.1)) #底盘尺寸 
        # self.chassis_shape_b = float(rospy.get_param('/chassis_shape_b',0.1))
        # self.chassis_wheel_r = float(rospy.get_param('/chassis_wheel_r',0.05))

        self.odom_pub = rospy.Publisher(self.odom_topic,Odometry,queue_size=10) 

        self.image_pub = rospy.Publisher(self.camera_image_topic, Image, queue_size=10)
        self.camInfo_pub = rospy.Publisher(self.camera_info_topic, CameraInfo, queue_size=10, latch=True)
        ros_pack = rospkg.RosPack()
        self.camera_info_path = open(ros_pack.get_path('esp_robot') + '/config/head_camera.yaml') #摄像头标定信息yaml文件

        self.imu_pub = rospy.Publisher(self.imu_topic,Imu, queue_size=10) 
        #控制话题订阅
        self.cmd_vel_sub = rospy.Subscriber(self.cmd_topic, Twist, self.cmd_vel_cb, queue_size=1) 

        self.br = tf.TransformBroadcaster() 
        self.bridge = CvBridge()

        self.client = socket(AF_INET, SOCK_STREAM)
    
        try:
            self.esp_cam = cv2.VideoCapture(self.esp_url)
        except:
            rospy.logerr('ERROR With Connect the cam')

        try:
            self.client.connect((self.esp_ctrl_url,self.esp_control_port)) 
        except:
            rospy.logerr("ERROR With Connect the control port")


        self.recv_pub_thread = threading.Thread(target=self.recv_pub_th)
        self.recv_pub_thread.start()

        self.make_camera_info()
        self.image_pub_thread = threading.Thread(target=self.image_pub_th)
        self.image_pub_thread.start()


    # Camera_info 数据包
    def make_camera_info(self):
        self.camera_info = CameraInfo()
        self.cam_data = yaml.safe_load(self.camera_info_path )

        self.camera_info.header = rospy.Header(stamp = rospy.Time.now())
        self.camera_info.header.frame_id = self.cam_data["camera_name"]

        self.camera_info.distortion_model = self.cam_data["distortion_model"]
        self.camera_info.width = self.cam_data['image_width']
        self.camera_info.height = self.cam_data['image_height']
        self.camera_info.binning_x = 0
        self.camera_info.binning_y = 0

        self.camera_info.K = self.cam_data['camera_matrix']['data']
        self.camera_info.D = self.cam_data['distortion_coefficients']['data']
        self.camera_info.R = self.cam_data['rectification_matrix']['data']
        self.camera_info.P = self.cam_data['projection_matrix']['data']
        self.camInfo_pub.publish(self.camera_info)

    #TCP 
    def recv_pub_th(self):
        while True:
            r =  self.client.recv(128)
            print(r.decode("utf-8"))

    #图像发布线程
    def image_pub_th(self):
        while not rospy.is_shutdown():
            rval = True
            while rval:
                try:
                    rval, frame  = self.esp_cam.read()
                    frame_filp = cv2.flip(frame,-1)
                    header = Header(stamp = rospy.Time.now())
                    header.frame_id = self.camera_info.header.frame_id
                    cv_img = self.bridge.cv2_to_imgmsg(frame_filp, "bgr8")
                    size = frame.shape
                    cv_img.header=header
                    cv_img.width = size[1]
                    cv_img.height = size[0]
                    cv_img.encoding = "bgr8"
                    cv_img.data = cv_img.data
                    self.camera_info.header = header
                    self.camInfo_pub.publish(self.camera_info)
                    self.image_pub.publish(cv_img) #发布消息
                    cv2.waitKey(1)
                except:
                    rospy.logerr("ERROR with getting img")

    def cmd_vel_cb(self, msg):
        strings = "123:123\n"
        self.client.send(strings.encode("utf-8"))
        #
        #self.move_with_chassis_control(msg.linear.x, msg.linear.y, -msg.angular.z * 57.29578) #

    def esp_exit(self):
        self.client.close()
        self.esp_cam.release()
        cv2.destroyAllWindows()



if __name__ == '__main__':

   rospy.init_node('esp_robot', anonymous=True)
   my_esp_robot = ESP_ROBOT()
   rospy.spin()
   my_esp_robot.esp_exit()

