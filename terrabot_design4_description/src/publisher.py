#!/usr/bin/env python3
# license removed for brevity
import rospy
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist

pub = rospy.Publisher('/Revolute_59_position_controller/command',
						Float64, queue_size=10)
pub1 = rospy.Publisher('/Revolute_60_position_controller/command',
						Float64, queue_size=10)
pub2 = rospy.Publisher('/mobile_base_controller/cmd_vel',
						Twist, queue_size=10)
pub3 = rospy.Publisher('/mobile_base_controller1/cmd_vel',
						Twist, queue_size=10)

rospy.init_node('terrabot_publisher', anonymous=True)
rate = rospy.Rate(10)

def Revolute_59(pub):
	position59 = 0.349
	rospy.loginfo(position59)
	pub.publish(position59)
	rate.sleep()

def Revolute_60(pub1):
	position60 = -0.349
	rospy.loginfo(position60)
	pub1.publish(position60)
	rate.sleep()


def Revolute_wheel_front(pub2):
	data1 = Twist()
	data1.linear.x = 30.0
	data1.linear.y = 0.0
	data1.linear.z = 0.0
	data1.angular.x = 0.0
	data1.angular.y = 0.0
	data1.angular.z = 0.0

	rospy.loginfo(data1)
	pub2.publish(data1)

def Revolute_wheel_rear(pub3):
	data2 = Twist()
	data2.linear.x = 30.0
	data2.linear.y = 0.0
	data2.linear.z = 0.0
	data2.angular.x = 0.0
	data2.angular.y = 0.0
	data2.angular.z = 0.0

	rospy.loginfo(data2)
	pub3.publish(data2)





if __name__ == '__main__':
	# it is good practice to maintain
	# a 'try'-'except' clause
	try:
		while not rospy.is_shutdown():
			Revolute_59(pub)
			Revolute_60(pub1)
			Revolute_wheel_front(pub2)
			Revolute_wheel_rear(pub3)
			rate.sleep()
	except rospy.ROSInterruptException:
		pass
