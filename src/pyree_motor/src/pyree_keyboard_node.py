#!/usr/bin/env python3

import rospy
from ackermann_msgs.msg import AckermannDriveStamped
from open_manipulator_msgs.msg import ArmToCar



move_state=False
forward_state = True

def arm_to_car_callback(data):
    global move_state
    global forward_state
    #rospy.loginfo("Received ArmToCar msg: arm_position=%.2f, car_position=%.2f", data.start_time, data.car_position)
    start_time = data.start_time
    move_state = data.move
    forward_state = data.forward_state 
    print(move_state)


if __name__ == "__main__":
    try:
        rospy.init_node("ackermann_publisher_and_subscriber", anonymous=True)
        rospy.Subscriber("/arm_to_car_topic", ArmToCar, arm_to_car_callback)
        pub =  rospy.Publisher("ackermann_cmd", AckermannDriveStamped, queue_size=2)
        
        rate = rospy.Rate(10) # 10hz
        
        ackermann_msg = AckermannDriveStamped()
        
        while not rospy.is_shutdown():
            if (move_state == True) and (forward_state==True) and (ackermann_msg.drive.speed == 0):
                print("iftrue")
                ackermann_msg.drive.speed = 10
                ackermann_msg.drive.steering_angle = 0
                pub.publish(ackermann_msg)
                rospy.loginfo("Publishing AckermannDriveStamped with speed: %f", ackermann_msg.drive.speed)
                
                
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
            rate.sleep()

    except rospy.ROSInterruptException:
        pass