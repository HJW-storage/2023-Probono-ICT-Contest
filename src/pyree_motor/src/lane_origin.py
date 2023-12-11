#!/usr/bin/env python3
# -*- coding: utf-8 -*-

#내가 수정한 파일 여기에 방지턱에 올라갔을때 감지된느 부분 추가돼있음. 
import numpy as np
import cv2
import rospy
import math
from sensor_msgs.msg import Image
from sensor_msgs.msg import Imu
from cv_bridge import CvBridge
#from xycar_msgs.msg import xycar_motor
import signal
import sys
import os
import time
import rospy
from ackermann_msgs.msg import AckermannDriveStamped
from open_manipulator_msgs.msg import ArmToCar

import threading 

sys.path.append("/catkin_ws/src/")
#=============================================
# 터미널에서 Ctrl-C 키입력으로 프로그램 실행을 끝낼 때
# 그 처리시간을 줄이기 위한 함수
#=============================================
def signal_handler(sig, frame):
    os.system('killall -9 python rosout')
    sys.exit(0)

signal.signal(signal.SIGINT, signal_handler)


#=============================================
# 주행을 위한 알고리즘 클래스 정의
#=============================================
class pipeline:

    #========================================
    # 변수 선언 및 초기화
    #========================================
    def __init__(self):
        self.cam=None
        rospy.Subscriber('usb_cam/image_raw', Image, self.callback_cam)
        rospy.Subscriber("/arm_to_car_topic", ArmToCar, self.arm_to_car_callback)
        self.angle=0
        self.speed=0
        self.move_state=False
        self.forward_state = True
        self.WIDTH, self.HEIGHT = 640, 480  # 카메라 가로, 세로 크기
        # 슬라이딩 윈도우 출력 창 크기 좌우 확장 값으로, 좌우로 window_margin 만큼 커짐
        # 슬라이딩 윈도우 출력 창 가로 크기 : WIDTH + 2*window_margin
        self.window_margin = 240
        #self.motor_msg = xycar_motor()      # xycar 제어를 위한 속도, 조향각 정보를 담고 있는 xycar_motor 호출
        self.bridge = CvBridge()            # To convert ros_image_msg type to numpy array type
        self.up_time = None

        # //==========
        # 기본(하단) ROI
        # 차선 방향에 따라 차량의 조향을 결정하기 위해 차량의 바로 앞쪽 영역인 하단 영역을 ROI로 설정
        # 순서 : np.array([좌하, 좌상, 우상, 우하])
        self.left_low = (0, self.HEIGHT - 80)
        self.left_top = (self.WIDTH // 2 - 250, self.HEIGHT // 2 + 90)
        self.right_low = (self.WIDTH, self.HEIGHT - 80)
        self.right_top = (self.WIDTH // 2 + 250, self.HEIGHT // 2 + 90)
        self.__vertices = np.array([self.left_low, self.left_top,
                                    self.right_top, self.right_low], dtype=np.int32)  # region of interest

        # Bird's eye View 변환을 위한 src, dst point 설정 (src 좌표에서 dst 좌표로 투시 변환)
        # 순서 : [좌하, 좌상, 우상, 우하]
        self.points_src = np.float32(list(self.__vertices))
        self.points_dst = np.float32([(100, self.HEIGHT), (100, 0), (self.WIDTH-100, 0), (self.WIDTH-100, self.HEIGHT)])

        # 만든 src, dst point를 이용하여 투시 변환 행렬 생성
        self.transform_matrix = cv2.getPerspectiveTransform(self.points_src, self.points_dst)
        # 원본 영상으로 되돌리기 위한 역변환 행렬
        self.inv_transfrom_matrix = cv2.getPerspectiveTransform(self.points_dst, self.points_src)
        
        #ros topic
   
        # 최종) 총 높이 95로 설정, 직진 중 차선이 일직선을 유지할 수 있도록 상단 좌표 2개 가로 너비를 약간 키움, 확장 ROI 설정 시 margin 값 기본(45)에서 50으로 증가
        self.__vertices_2 = np.array([(0, self.HEIGHT - 50), (self.WIDTH // 2 - 142, self.HEIGHT - 145),
                                    (self.WIDTH // 2 + 142, self.HEIGHT - 145), (self.WIDTH, self.HEIGHT - 50)], dtype=np.int32)  # region of interest

        # Bird's eye View 변환을 위한 src, dst point 설정 (src 좌표에서 dst 좌표로 투시 변환)
        # 순서 : [좌하, 좌상, 우상, 우하]
        self.points_src2 = np.float32(list(self.__vertices_2))
        self.points_dst2 = np.float32([(100, self.HEIGHT), (100, 0), (self.WIDTH-100, 0), (self.WIDTH - 100, self.HEIGHT)])

        # 만든 src, dst point를 이용하여 투시 변환 행렬 생성
        self.transform_matrix_2 = cv2.getPerspectiveTransform(self.points_src2, self.points_dst2)  # 변환 행렬
        # 원본 영상으로 되돌리기 위한 역변환 행렬
        self.inv_transfrom_matrix_2 = cv2.getPerspectiveTransform(self.points_dst2, self.points_src2)  # 역변환 행렬
        # ==========//

        self.sel = 0  # 기본(하단) or 확장(상단) ROI를 선택하기 위한 변수

        self.leftx_mid, self.rightx_mid = self.WIDTH//4, self.WIDTH*3//4  # 슬라이딩 윈도우 기준점 초기 좌표
        self.leftx_base, self.rightx_base = self.leftx_mid, self.rightx_mid  # 슬라이딩 윈도우 이전값

        self.left_a, self.left_b, self.left_c = [], [], []      # 왼쪽 차선으로부터 나온 2차 곡선 방정식의 계수를 저장하기 위한 변수
        self.right_a, self.right_b, self.right_c = [], [], []   # 오른쪽 차선으로부터 나온 2차 곡선 방정식의 계수를 저장하기 위한 변수
        
        # 양쪽 차선 곡선 좌표 생성
        ### Linear y 값 생성 (0, 1, 2, ..., 479)
        self.ploty = np.linspace(0, self.HEIGHT - 1, self.HEIGHT)

        # 이전 조향각을 기억하기 위한 변수 선언 및 0으로 초기화
        self.prev_angle = 0

        self.ros_init()  # ros 초기화


    #========================================
    # ROS 설정
    #========================================
    def ros_init(self):
        rospy.init_node('xytron_driving')  # 노드 정의
        self.rate = rospy.Rate(20)  # 30Hz로 토픽 발행
        self.pub =  rospy.Publisher("ackermann_cmd", AckermannDriveStamped, queue_size=2)
        rospy.loginfo(rospy.get_name() + " started!")  # 노드 네임 출력


    #========================================
    def white_detection(self, img):
        img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)  # 색상 영역을 HSV로 변환
        lower_white = np.array([0, 0, 180], dtype=np.uint8)  # 흰색 픽셀 하한 값
        upper_white = np.array([255, 70, 255], dtype=np.uint8)  # 흰색 픽셀 상한 값
        mask = cv2.inRange(img, lower_white, upper_white)  # lower와 upper 범위내에 존재하는 픽셀 추출
        img = cv2.bitwise_and(img, img, mask=mask)  # 비트 AND연산
        return img  # 이미지 반환


    #========================================
    def gray_scale(self, img):  
        return cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)  # 그레이 스케일 이미지로 변경하여 이미지 반환


    #========================================
    def gaussian_blur(self, img):  
        return cv2.GaussianBlur(img, (5, 5), 0)  # 노이즈 제거(솔트 & 페퍼 노이즈) 이미지 반환

  

    #========================================
    def morphology_close(self, img): 
        kernel = np.ones((5, 5))  # mask 크기를 5 * 5로 설정
        return cv2.morphologyEx(img, cv2.MORPH_CLOSE, kernel)  # 모폴로지 닫기 연산(차선을 깨끗하게 하기 위해서)


    #========================================
    # Bird's eye View 변환

    #========================================
    def perspective_transform(self, img, sel=0):  # birds eye view
        if sel == 0:
            result_img = cv2.warpPerspective(img, self.transform_matrix, (self.WIDTH, self.HEIGHT))  # 기본(하단) ROI로 변환 행렬 구하기
        else:
            result_img = cv2.warpPerspective(img, self.transform_matrix_2, (self.WIDTH, self.HEIGHT))  # 확장(상단) ROI로 변환 행렬 구하기
        return result_img  # 변환한 이미지 반환


    def sliding_window(self, img, nwindows=15, margin=35, minpix=50, draw_windows=True):  # sliding window
        # 크기 3의 비어있는 배열 생성
        left_fit_ = np.empty(3)
        right_fit_ = np.empty(3)

        # 0과 1로 이진화된 영상을 3채널의 영상으로 만들기 위해 3개를 쌓은 후 *255
        out_img = np.dstack((img, img, img)) * 255

        # 슬라이딩 윈도우를 출력하기 위한 가로로 긴 이미지를 추가로 만듦
        # 차선이 ROI 영역 밖으로 넘어가며 슬라이딩 윈도우도 함께 잘려서 출력되는 것을 방지하고자 함
        window_img = np.zeros((self.HEIGHT, self.WIDTH+2*self.window_margin, 3))
        # 인식된 차선 이미지를 담고 있는 out_img를 window_img의 중앙에 동일하게 가져옴
        window_img[:, self.window_margin:self.window_margin+self.WIDTH, :] = out_img

        midpoint = self.WIDTH // 2

        # 조사창의 높이 설정
        # 전체 높이에서 설정한 조사창 개수만큼 나눈 값
        window_height = self.HEIGHT // nwindows

        # 0이 아닌 픽셀의 x,y 좌표 반환 (흰색 영역(차선)으로 인식할 수 있는 좌표)
        nonzero = img.nonzero()
        nonzeroy = np.array(nonzero[0])
        nonzerox = np.array(nonzero[1])

        leftx_current = self.leftx_base
        rightx_current = self.rightx_base


        leftx_past = leftx_current
        rightx_past = rightx_current


        left_lane_inds = []
        right_lane_inds = []

        # 설정한 조사창 개수만큼 슬라이딩 윈도우 생성
        for window in range(nwindows):
            # 조사창 크기 및 위치 설정
            win_y_low = self.HEIGHT - ((window + 1) * window_height)
            # n번째 조사창 윗변 y 좌표 : (전체 높이) - (n * 조사창 높이)
            win_y_high = self.HEIGHT - (window * window_height)
            # 양쪽 차선의 조사창의 너비를 현재 좌표로부터 margin만큼 양 옆으로 키움
            win_xleft_low = leftx_current - margin
            win_xleft_high = leftx_current + margin
            win_xright_low = rightx_current - margin
            win_xright_high = rightx_current + margin

            # 조사창 그리기
            if draw_windows == True:
                # 왼쪽 차선
                cv2.rectangle(out_img, (win_xleft_low, win_y_low), (win_xleft_high, win_y_high), (100, 100, 255), 1)
                # 오른쪽 차선
                cv2.rectangle(out_img, (win_xright_low, win_y_low), (win_xright_high, win_y_high), (100, 100, 255), 1)

                cv2.rectangle(window_img, (win_xleft_low+self.window_margin, win_y_low), (win_xleft_high+self.window_margin, win_y_high), (255, 100, 100), 1)
                # 오른쪽 차선
                cv2.rectangle(window_img, (win_xright_low+self.window_margin, win_y_low), (win_xright_high+self.window_margin, win_y_high), (255, 100, 100), 1)

            # 조사창 내부에서 0이 아닌 픽셀의 인덱스 저장
            good_left_inds = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high)
                             & (nonzerox >= win_xleft_low) & (nonzerox < win_xleft_high)).nonzero()[0]
            good_right_inds = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high)
                             & (nonzerox >= win_xright_low) & (nonzerox < win_xright_high)).nonzero()[0]

            # 양쪽 차선의 인덱스 저장
            left_lane_inds.append(good_left_inds)
            right_lane_inds.append(good_right_inds)

            # 조사창 내부에서 0이 아닌 픽셀 개수가 기준치를 넘으면 해당 픽셀들의 인덱스 평균값(x좌표 평균)으로 다음 조사창의 위치(x좌표)를 결정
            if len(good_left_inds) > minpix:
                leftx_current = np.int(np.mean(nonzerox[good_left_inds]))
            if len(good_right_inds) > minpix:
                rightx_current = np.int(np.mean(nonzerox[good_right_inds]))

            # 양쪽 차선 중 하나만 인식된 경우 반대편 차선에서 나타난 인덱스 변화량과 동일하게 인덱스 설정
            # 인식된 차선의 방향과 동일하게 그려짐
            if len(good_left_inds) < minpix:
                leftx_current = leftx_current + (rightx_current-rightx_past)
            if len(good_right_inds) < minpix:
                rightx_current = rightx_current + (leftx_current-leftx_past)

 
            if window == 0:
                # 왼쪽 차선의 기준점이 중앙 기준 우측으로 넘어가지 않도록 제한
                if leftx_current > midpoint+40:
                    leftx_current = midpoint+40
                # 왼쪽 차선의 기준점이 왼쪽 화면 밖으로 나가지 않도록 제한
                if leftx_current < 0:
                    leftx_current = 0

                # 오른쪽 차선의 기준점이 중앙 기준 좌측으로 넘어가지 않도록 제한
                if rightx_current < midpoint-40:
                    rightx_current = midpoint-40
                # 오른쪽 차선의 기준점이 오른쪽 화면 밖으로 나가지 않도록 제한
                if rightx_current > self.WIDTH:
                    rightx_current = self.WIDTH
                    
                # 두번째 조사창의 현재 좌표를 다음 프레임의 기준점으로 설정
                self.leftx_base = leftx_current
                self.rightx_base = rightx_current

            leftx_past = leftx_current
            rightx_past = rightx_current

        # 배열 연결
        left_lane_inds = np.concatenate(left_lane_inds)
        right_lane_inds = np.concatenate(right_lane_inds)


        leftx = nonzerox[left_lane_inds]
        lefty = nonzeroy[left_lane_inds]
        ### 0이 아닌 픽셀 중에서 오른쪽 차선으로 인식된 좌표만 가져옴
        rightx = nonzerox[right_lane_inds]
        righty = nonzeroy[right_lane_inds]

        # 차선으로 인식된 픽셀 수가 일정치 이상일 경우에만 차선이 인식된 것으로 판단
        ### 왼쪽 차선으로 인식된 좌표가 5000개 미만이라면 False, 이상이라면 True
        if (leftx.size < 5000):
            left_lane_detected = False
        else:
            left_lane_detected = True
        ### 오른쪽 차선으로 인식된 좌표가 5000개 미만이라면 False, 이상이라면 True
        if (rightx.size < 5000):
            right_lane_detected = False
        else:
            right_lane_detected = True

        if left_lane_detected:
            # 검출된 차선 좌표들을 통해 왼쪽 차선의 2차 방정식 계수를 구함
            left_fit = np.polyfit(lefty, leftx, 2)

            # 왼쪽 차선 계수
            self.left_a.append(left_fit[0])
            self.left_b.append(left_fit[1])
            self.left_c.append(left_fit[2])

        ### 오른쪽 차선이 인식된 경우
        if right_lane_detected:
            # 검출된 차선 좌표들을 통해 오른쪽 차선의 2차 방정식 계수를 구함
            right_fit = np.polyfit(righty, rightx, 2)

            # 오른쪽 차선 계수
            self.right_a.append(right_fit[0])
            self.right_b.append(right_fit[1])
            self.right_c.append(right_fit[2])


        out_img[nonzeroy[left_lane_inds], nonzerox[left_lane_inds]] = [255, 0, 0]
        out_img[nonzeroy[right_lane_inds], nonzerox[right_lane_inds]] = [0, 0, 255]
        # window_img : 양쪽으로 각각 window_margin 만큼 넓힌 이미지이므로 x좌표를 window_margin 만큼 이동시켜 그려주어야 함
        window_img[nonzeroy[left_lane_inds], nonzerox[left_lane_inds]+self.window_margin] = [255, 0, 0]
        window_img[nonzeroy[right_lane_inds], nonzerox[right_lane_inds]+self.window_margin] = [0, 0, 255]

        # 계수마다 각각 마지막 10개의 평균으로 최종 계수 결정
        ### 왼쪽 차선의 계수 결정
        left_fit_[0] = np.mean(self.left_a[-10:])
        left_fit_[1] = np.mean(self.left_b[-10:])
        left_fit_[2] = np.mean(self.left_c[-10:])
        ### 오른쪽 차선의 계수 결정
        right_fit_[0] = np.mean(self.right_a[-10:])
        right_fit_[1] = np.mean(self.right_b[-10:])
        right_fit_[2] = np.mean(self.right_c[-10:])

        # y 값에 해당하는 x 값 결정
        ### 왼쪽 차선
        left_fitx = left_fit_[0] * self.ploty ** 2 + left_fit_[1] * self.ploty + left_fit_[2]

        ### 오른쪽 차선
        right_fitx = right_fit_[0] * self.ploty ** 2 + right_fit_[1] * self.ploty + right_fit_[2]

        # 양쪽 모두 차선인식이 안됐다면 슬라이딩 윈도우 조사창 재설정
        if(left_lane_detected is False) and (right_lane_detected is False):
            self.leftx_base = self.leftx_mid 
            self.rightx_base = self.rightx_mid -6
        
        # 출력 이미지, 양쪽 곡선 x 좌표, 차선 인식 여부 반환
        return out_img, window_img, left_fitx, right_fitx, left_lane_detected, right_lane_detected

    def inv_perspective_transform(self, img, sel=0):
        if sel == 0:
            result_img = cv2.warpPerspective(img, self.inv_transfrom_matrix, (self.WIDTH, self.HEIGHT))
        else:
            result_img = cv2.warpPerspective(img, self.inv_transfrom_matrix_2, (self.WIDTH, self.HEIGHT))
        return result_img

    def combine_img(self, origin_img, result_img):
        return cv2.addWeighted(origin_img, 0.5, result_img, 1.0, 0.8)


    def get_angle_on_lane(self, left_fitx, right_fitx, left_lane_detected, right_lane_detected):
        # 두 차선 인식 모두 안되는 경우 조향각: 0, 속도: 20 반환
        if (left_lane_detected is False) and (right_lane_detected is False):
            return self.prev_angle, self.calc_speed(self.prev_angle)
        
        # 양쪽 차선 중 하나만 인식된 경우
        elif (left_lane_detected is False) or (right_lane_detected is False):
            # 왼쪽 차선이 인식된 경우 왼쪽 차선의 기울기를 그대로 적용
            if left_lane_detected is True:
                left_lane = list(np.polyfit(left_fitx, self.ploty, deg=1))  # 왼쪽 차선 좌표로부터 직선 방정식의 계수와 절편을 구함
                slope = left_lane[0]  # 나온 기울기 값 저장
            
            # 오른쪽 차선이 인식된 경우 오른쪽 차선의 기울기를 그대로 적용
            else:
                right_lane = list(np.polyfit(right_fitx, self.ploty, deg=1))  # 오른쪽 차선 좌표로부터 직선 방정식의 계수와 절편을 구함
                slope = right_lane[0]  # 나온 기울기 값 저장

        # 양쪽 차선이 모두 인식된 경우
        else:
            left_lane = list(np.polyfit(left_fitx, self.ploty, deg=1))  # 왼쪽 차선 직선 방정식의 계수와 절편을 구함
            right_lane = list(np.polyfit(right_fitx, self.ploty, deg=1))  # 오른쪽 차선 직선 방정식의 계수와 절편을 구함


            if left_lane[0] == right_lane[0]:
                Inter_X = -(right_lane[1] + left_lane[1]) / (2 * left_lane[0])
                Inter_Y = 0

            else: # 두 차선의 기울기가 평행하지 않은 경우 소실점 추출
                Inter_X = (right_lane[1] - left_lane[1]) / (left_lane[0] - right_lane[0])  # 두 직선 방정식의 교차 X점 구하기
                Inter_Y = np.poly1d(left_lane)(Inter_X)  # 두 직선 방정식의 교차 Y점 구하기
                print('Inter_Y = ',Inter_Y)
            
            slope = (self.HEIGHT - Inter_Y) / ((self.WIDTH / 2) - Inter_X)  # 카메라 하단 중앙점과 차선 상단 중앙점 사이의 기울기 차이를 구하기
        

        steering_angle = math.atan(slope) * 180 / math.pi  # 조향각 추출

        if steering_angle > 0:
           steering_angle = steering_angle - 90
        
        elif steering_angle < 0:
           steering_angle = steering_angle + 90
        
        else:
           pass

        # 소실점 기준으로 계산된 각도 범위를 -22도 ~ 22도로 제한
        # change
        if steering_angle >= 27:
            steering_angle = 27

        elif steering_angle <= -27:
            steering_angle = -27
        
        else:
            pass

        #print('Original Angle: {}'.format(steering_angle))  # 계산된 조향각 출력

        weighted_angle = self.calc_angle_weights(steering_angle)    # 가중치가 부여된 angle
        self.prev_angle = weighted_angle                            # 현재 조향각을 이전 조향각 변수에 저장
        #print("Weighted Angle: {}".format(weighted_angle))          # 조향각 출력
        
        speed = self.calc_speed(weighted_angle)  # 각도에 따른 차량 속도 구하기
        
        return weighted_angle, speed  # 조향각, 속도 반환
       

    #========================================
    #========================================
    def calc_angle_weights(self, angle): # 조향각에 따른 조향각 가중치 곱함
        # 각도에 따른 가중치 설정
        if abs(angle) < 5.0:
            weight = 1.2
        
        elif abs(angle) < 10.0:
            weight = 0.95

        elif abs(angle) < 15.0:
            weight = 0.82

        elif abs(angle) < 20.0:
            weight = 0.66

        elif abs(angle) <= 25.0:
            weight = 0.5

        else:
            weight = 1.00

        weighted_angle = weight * angle  # 가중치 * 조향각 = 증폭된 조향각

        # 조향각 범위 -22 ~ 22으로 제한
        if weighted_angle >= 22.0:
            weighted_angle = 22.0
        
        elif weighted_angle <= -22.0:
            weighted_angle = -22.0
        
        else:
            pass
        
        return np.round(weighted_angle , 1)  # 소숫점 첫째자리에서 반올림


    #========================================
    #========================================
    def calc_speed(self, angle):
        if angle < -0.8:
            slope = 0.9
        elif angle > 0.8:
            slope = -0.9
        else:
            slope = 0
        
        # 속도 계산
        speed = np.round((slope * angle) + 25, 1) #  기울기가 클수록 속도 감속 
        #print("Current Speed: {}".format(speed))  # 속도 출력
        print("")
        
        return speed  # 속도 반환


    #========================================
    # speed : 속도
    #========================================
    def motor_publisher(self, angle, speed):
        self.motor_msg.angle = angle  # 각도
        self.motor_msg.speed = speed  # 속도
        print("Pub Speed: {}".format(speed))  # 속도 출력
        print("Pub Angle: {}".format(angle))
        self.motor_pub.publish(self.motor_msg)  # Publish motor Angle, Speed
        self.rate.sleep()  # ros 초기 설정 시 설정한 주기에 맞게 전송하기 위한 딜레이

    def callback_cam(self, msg):
        self.cam = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        
        #start_time = time.time()
        self.cam = cv2.resize(self.cam, (640, 480))
        #print('time2 : ',time.time()-start_time)
        
        #cv2.imshow('real_cam', self.cam)
        #cv2.waitKey(1)
        self.processing(self.cam)
        cv2.waitKey(1)
        
    def arm_to_car_callback(self,data):
        #rospy.loginfo("Received ArmToCar msg: arm_position=%.2f, car_position=%.2f", data.start_time, data.car_position)
        start_time = data.start_time
        self.move_state = data.move
        self.forward_state = data.forward_state 
        print(self.move_state)
        
    #========================================
    # Main 함수
    # =====================
    # img : 입력 이미지 (ros 토픽으로부터 받은 카메라 입력 이미지)
    #========================================
    def processing(self, img):
        #image = self.bridge.imgmsg_to_cv2(img, "bgr8") # ros_image_msg type을 numpy array로 변환
        #img = image.copy()  # 원본 데이터 복사

        self.sel = 0  # 기본(하단) ROI로 설정하기 위한 값 설정
        white_threshold_img = self.white_detection(img)  # 흰색 영역 검출
        gaussian_blurred_img = self.gaussian_blur(white_threshold_img)  # 노이즈 제거를 위한 가우시안 블러
        thres_img = self.gray_scale(gaussian_blurred_img)  # 그레이 스케일 이미지 변환
        closed_img = self.morphology_close(thres_img)  # 모폴로지 닫힘 연산, 선명한 엣지 검출을 위해 적용
        birds_eye_viewed_img = self.perspective_transform(closed_img, sel=0)  # 원근감을 제거하기 위한 버드아이뷰 적용
        #start_time = time.time()
        sliding_window_img, window_img, left_fitx, right_fitx, left_lane_detected, right_lane_detected = self.sliding_window(birds_eye_viewed_img)  # 슬라이딩 윈도우로 곡선 차선 인식
        #print('time : ',time.time()-start_time)
        # 양쪽 차선이 모두 인식이 되지 않은 경우, 도로 영역의 상단으로 ROI 설정
        if (left_lane_detected is False) and (right_lane_detected is False):
            self.sel = 1  # 확장(상단) ROI로 설정
            birds_eye_viewed_img = self.perspective_transform(closed_img, sel=self.sel)

            sliding_window_img, window_img, left_fitx, right_fitx, left_lane_detected, right_lane_detected = self.sliding_window(birds_eye_viewed_img, margin=50)

        
        self.angle, self.speed = self.get_angle_on_lane(left_fitx, right_fitx, left_lane_detected, right_lane_detected)  # 인식된 차선의 좌표값을 기준으로 각도를 구함
        
        
        
        inv_transformed_img = self.inv_perspective_transform(sliding_window_img, sel=self.sel)  # 버드아이뷰 이미지를 다시 원래 이미지로 되돌리기 위한 역변환
        combined_img = self.combine_img(img, inv_transformed_img)  # 원본이미지와 프로세싱된 이미지를 합성
        print('angle, speed = ', self.angle, self.speed)
        print(rospy.Time.now())

        #self.motor_publisher(angle, speed) # 모터 제어 조향각, 속도값 퍼블리싱

        #cv2.imshow('Sliding Window2', window_img)  # 슬라이딩 윈도우 출력
        combined_img=cv2.line(combined_img,self.left_low,self.left_top,(0,255,0),5)
        combined_img=cv2.line(combined_img,self.right_low,self.right_top,(0,255,0),5)
        cv2.imshow('CAM View', combined_img)  # 원본 영상에 차선 인식 결과 출력


#=============================================
# main 함수
#=============================================
if __name__ == '__main__':
    try:
        drive = pipeline()  # 주행을 위한 pipeline 클래스 호출
        ackermann_msg = AckermannDriveStamped()
        # spin_thread = threading.Thread(target=spin_in_thread)
        # spin_thread.start()
        
        while not rospy.is_shutdown():
            if (drive.move_state == True) and (drive.forward_state==True) :
                print("iftrue")
                ackermann_msg.drive.speed = drive.speed
                ackermann_msg.drive.steering_angle = drive.angle
                drive.pub.publish(ackermann_msg)
                rospy.loginfo("Publishing AckermannDriveStamped with speed: %f", ackermann_msg.drive.speed)
                rospy.loginfo("Publishing AckermannDriveStamped with angle: %f", ackermann_msg.drive.steering_angle)
                
            elif drive.move_state == False and (ackermann_msg.drive.speed != 0) :
                print("iffalse")
                ackermann_msg.drive.speed = 0
                ackermann_msg.drive.steering_angle = 0
                drive.pub.publish(ackermann_msg)
                rospy.loginfo("Publishing AckermannDriveStamped with speed: %f", ackermann_msg.drive.speed)
            drive.rate.sleep()

    except Exception as ex:
            print(ex)