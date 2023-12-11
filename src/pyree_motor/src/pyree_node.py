# #!/usr/bin/env python3

import rospy
from ackermann_msgs.msg import AckermannDriveStamped
from open_manipulator_msgs.msg import ArmToCar
import signal
import sys
import os
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import sys
sys.path.append("/catkin_ws/src/")
from pyree_motor.src.lanedetection import pipeline

# from lanedetection import pipeline


move_state=False
forward_state = True
def signal_handler(sig, frame):
    os.system('killall -9 python rosout')
    sys.exit(0)

signal.signal(signal.SIGINT, signal_handler)

def arm_to_car_callback(data):
    global move_state
    global forward_state
    #rospy.loginfo("Received ArmToCar msg: arm_position=%.2f, car_position=%.2f", data.start_time, data.car_position)
    start_time = data.start_time
    move_state = data.move
    forward_state = data.forward_state 
    print(move_state)
    
def callback_cam(msg):
    cam = CvBridge.imgmsg_to_cv2(msg, "bgr8")
    cv2.imshow('real_cam', cam)


if __name__ == "__main__":
    
    try:
        rospy.init_node("ackermann_publisher_and_subscriber", anonymous=True)
        drive = pipeline()  # 주행을 위한 pipeline 클래스 호출
        rospy.Subscriber('/usb_cam/image_raw', Image, drive.callback_cam)
        rospy.Subscriber("/arm_to_car_topic", ArmToCar, arm_to_car_callback)
        pub =  rospy.Publisher("ackermann_cmd", AckermannDriveStamped, queue_size=2)    
        rate = rospy.Rate(10) # 10hz
        ackermann_msg = AckermannDriveStamped()
        while not rospy.is_shutdown():
            if (move_state == True) and (forward_state==True) and (ackermann_msg.drive.speed == 0):
                print("iftrue")
                angle, speed = drive.processing(drive.cam)
                ackermann_msg.drive.speed = speed
                ackermann_msg.drive.steering_angle = angle
                pub.publish(ackermann_msg)
                rospy.loginfo("Publishing AckermannDriveStamped with speed: %f", ackermann_msg.drive.speed)
                rospy.loginfo("Publishing AckermannDriveStamped with angle: %f", ackermann_msg.drive.steering_angle)
                
                
            elif move_state == False and (ackermann_msg.drive.speed != 0) :
                print("iffalse")
                ackermann_msg.drive.speed = 0
                ackermann_msg.drive.steering_angle = 0
                pub.publish(ackermann_msg)
                rospy.loginfo("Publishing AckermannDriveStamped with speed: %f", ackermann_msg.drive.speed)
                
            elif (move_state == True) and (forward_state==False) and (ackermann_msg.drive.speed == 0):
                print("iffalse")
                ackermann_msg.drive.speed = -10
                ackermann_msg.drive.steering_angle = 0
                pub.publish(ackermann_msg)
                rospy.loginfo("Publishing AckermannDriveStamped with speed: %f", ackermann_msg.drive.speed)    
            rospy.spin()
            rate.sleep()

    except rospy.ROSInterruptException:
#         pass
if __name__ == '__main__':
    #rospy.init_node("ackermann_publisher_and_subscriber", anonymous=True)
    drive = pipeline()  # 주행을 위한 pipeline 클래스 호출
    rospy.Subscriber('/usb_cam/image_raw', Image, drive.callback_cam)  # ros 토픽으로 이미지를 받아 주행 프로세스 실행
    rospy.spin()  # 반복 실행

