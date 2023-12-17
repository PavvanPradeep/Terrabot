#!/usr/bin/env python3
# license removed for brevity

import rospy
from sensor_msgs.msg import Imu



def print_mssg(data):
    print("new Data received:(%d , %f, %f, %f ,%f)", data.angular_velocity ,data.linear_acceleration,data.orientation)


    
rospy.init_node('iot_sensor_subscriber_node', anonymous=True)

rospy.Subscriber("imu_topic_1",Imu, print_mssg)

# spin() simply keeps python from exiting until this node is stopped
rospy.spin()