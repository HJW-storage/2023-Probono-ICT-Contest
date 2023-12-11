#!/usr/bin/env python
# -*- coding:utf-8 -*-
import cv2
import rospy
from ackermann_msgs.msg import AckermannDriveStamped
from time import time
from time import sleep
from horse_power_sensor import HPSensor
from collections import deque

from open_manipulator_msgs.msg import ArmToCar
from controller import Stanley
#from lane_detector_jw import Camera
from lane_detector2 import LaneDetector
from obstacle_detector import Clustering
from traffic import traffic

class HP:

    def __init__(self):
        self.rate = rospy.Rate(20)  # 20Hz로 토픽 발행
        self.motor_pub = rospy.Publisher('ackermann_cmd', AckermannDriveStamped, queue_size=20)
        rospy.Subscriber("/arm_to_car_topic", ArmToCar, self.callback_ArmToCar)
        self.motor_msg = AckermannDriveStamped()  # 제어를 위한 속도, 조향각 정보를 담고 있는 ackermann_cmd 호출
        #self.tra=traffic()
        self.sensor = HPSensor()
        self.sensor.init(self.rate)
       
        #self.lane_detector = Camera()
        self.lane_detector = LaneDetector()
        self.obstacle_detector = Clustering()
        self.stanley = Stanley()
        self.start_time = time()
        self.que_speed = deque()
        self.que_angle = deque()
        self.move=True

    def callback_ArmToCar(self, msg):
        self.start_time = msg.start_time
        self.move = msg.move

        print(self.move)   
        

    def calc_speed(self, angle):  # 최저속도(min): 10.0, 최고속도: 50.0(50.0)
        if angle > 0:
            slope = -0.23

        elif angle < 0:
            slope = 0.23

        else:
            slope = 0.0
       
        speed = (slope * angle) + 63.0

        return speed

    def control(self):
        if self.move == True :
            self.motor_msg.drive.speed = 10
            self.motor_msg.drive.steering_angle = 0
        elif self.move == False :
            self.motor_msg.drive.speed = -10
            self.motor_msg.drive.steering_angle = 0

        self.motor_pub.publish(self.motor_msg) #if 문 안에넣기 또는 밖에넣기 골라야함
        self.rate.sleep()