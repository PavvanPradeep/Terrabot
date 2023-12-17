#!/usr/bin/env python3
# license removed for brevity
  
import rospy
from geometry_msgs.msg import Twist
  
  
def callback(data):
    rospy.loginfo("Received linear velocity in x-direction : %f",data.linear.x)
    rospy.loginfo("Received linear velocity in y-direction : %f",data.linear.y)
    rospy.loginfo("Received linear velocity in z-direction : %f",data.linear.z)
    rospy.loginfo("Received angular velocity in x-direction : %f",data.angular.x)
    rospy.loginfo("Received angular velocity in y-direction : %f",data.angular.y)
    rospy.loginfo("Received angular velocity in z-direction : %f",data.angular.z)
    
      
    # print the actual message in its raw format

      
    # otherwise simply print a convenient message on the terminal
    print('Data from /cmd_vel received')
  
  
def main():
      
    # initialize a node by the name 'listener'.
    # you may choose to name it however you like,
    # since you don't have to use it ahead
    rospy.init_node('terrabot_subscriber', anonymous=True)
    rospy.Subscriber("/cmd_vel", Twist, callback)
      
    # spin() simply keeps python from
    # exiting until this node is stopped
    rospy.spin()
  
if __name__ == '__main__':
      
    # you could name this function
    try:
        main()
    except rospy.ROSInterruptException:
        pass