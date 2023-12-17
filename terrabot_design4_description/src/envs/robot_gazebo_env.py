#!/usr/bin/env python3
# license removed for brevity
import gym
import rospy
import numpy as np
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from gazebo_msgs.srv import SetModelState
from gazebo_msgs.msg import ModelState
from gym.utils import seeding
from gym import spaces
from math import pow,sqrt
from std_msgs.msg import Float64

# rate = rospy.Rate(10)

class GazeboEnv(gym.Env):
    def __init__(self):
        rospy.init_node('terrabot_publisher', anonymous=True)
        # self.rate = rospy.Rate(10)
        self.model_state_msg = ModelState()
        self.model_state_msg.model_name = 'Terrabot'
        self.model_state_msg.pose.position.x = 0.0
        self.model_state_msg.pose.position.y = 0.0
        self.model_state_msg.pose.position.z = 0.0
        self.model_state_msg.pose.orientation.x = 0.0
        self.model_state_msg.pose.orientation.y = 0.0
        self.model_state_msg.pose.orientation.z = 0.0
        self.model_state_msg.pose.orientation.w = 1.0
        self.learning_rate = 0.5
        self.discount_factor = 0.5
        self.action_space = spaces.Discrete(4)
        self
        self.observation_space = spaces.Box(low=0, high=10, shape=(4,))
        self.rate = rospy.Rate(10)
        self.box_x = 0
        self.box_y = -1
        self.box_z = 0
        self.position_x = 0  # init coordinates
        self.position_y = 0
        self.position_z = 0
        self.episodes = 5
        self.actions_array = []
        self.pub = rospy.Publisher('/Revolute_59_position_controller/command',
						Float64, queue_size=10)
        self.pub1 = rospy.Publisher('/Revolute_60_position_controller/command',
						Float64, queue_size=10)
        self.pub2 = rospy.Publisher('/mobile_base_controller/cmd_vel',
						Twist, queue_size=10)
        self.pub3 = rospy.Publisher('/mobile_base_controller1/cmd_vel',
						Twist, queue_size=10)
        self.alpha=0.5
        self.beta=0.5
        self.epsilon=1
        self.gamma=1
        self.q_table = np.zeros((10, 10, 4))
        self.reward_val = 0
        

    def Revolute_59(self, pub):
        position59 = 0.349
        rospy.loginfo(position59)
        pub.publish(position59)
        self.rate.sleep()

    def Revolute_60(self, pub1):
        position60 = -0.349
        rospy.loginfo(position60)
        pub1.publish(position60)
        self.rate.sleep()
    
    def Revolute_wheel_front(self, pub2):
        self.rate = rospy.Rate(10)
        data1 = Twist()
        data1.linear.x = 0.0
        data1.linear.y = 0.0
        data1.linear.z = 0.0
        data1.angular.x = 0.0
        data1.angular.y = 0.0
        data1.angular.z = 30.0

        rospy.loginfo(data1)
        pub2.publish(data1)
    
    def Revolute_wheel_rear(self, pub3):
        data2 = Twist()
        data2.linear.x = 0.0
        data2.linear.y = 0.0
        data2.linear.z = 0.0
        data2.angular.x = 0.0
        data2.angular.y = 0.0
        data2.angular.z = 30.0

        rospy.loginfo(data2)
        pub3.publish(data2)

    
     
    def move(self, linear_vel, angular_vel):
        velocity_msg = Twist()
        velocity_msg.linear.x = linear_vel
        velocity_msg.angular.z = angular_vel
        self.pub2.publish(velocity_msg)   
        self.pub3.publish(velocity_msg) 
       
    def reset(self):
        self.position_x = 0
        self.position_y = 0  
        self.position_z = 0    
        return self.get_obs()

    def get_obs(self):
        return [self.box_x, self.box_y, self.box_z, self.position_x, self.position_y, self.position_z]
    
    def step(self,actions_array):
        self.move_robot()
        self.rate.sleep()
        obs = self.get_obs()
        reward = self.reward()
        done=self.done(self.position_x,self.box_x,self.position_y,self.box_y)
        return obs,reward,done,{}
    
    def reward(self):
        
        self.distance=sqrt((self.position_x-self.box_x)*2+(self.position_y-self.box_y)*2)
        self.reward_val = -self.distance
        return self.reward_val

    def move_robot(self):
        # if action == 0:
        # # Apply the desired action for joint 59 (Revolute_59)
        #     self.Revolute_59(self.pub)
        # # Set the desired wheel velocities
        # #self.move_wheels(0, 0)
        
        # if action == 1:
        # # Apply the desired action for joint 60 (Revolute_60)
        #     self.Revolute_60(self.pub1)
        # # Set the desired wheel velocities
        # #self.move_wheels(0, 1)

        # if action == 2:
        #     self.Revolute_wheel_front(self.pub2)
        
        # if action == 3:
        #     self.Revolute_wheel_rear(self.pub3)
        
        self.actions_array.append(self.Revolute_59(self.pub))
        self.actions_array.append(self.Revolute_60(self.pub1))
        self.actions_array.append(self.Revolute_wheel_front(self.pub2))
        self.actions_array.append(self.Revolute_wheel_rear(self.pub3))

        return self.actions_array

    
    
    def done(self,x,y,box_x,box_y):
        self.distance=sqrt((self.position_x-self.box_x)*2+(self.position_y-self.box_y)*2)
        if self.distance==0:
            if x==box_x and y==box_y:
                return True
    
    def get_state_index(self, state):       
        x = int(state[2])  # position_x
        y = int(state[3])  # position_y
        return x, y
    
        
    def get_action(self, state_index):
        # x, y = state_index
        # if np.random.random() < self.epsilon:
        #     return np.argmax(self.q_table[x,y])
        # else:
        #     return np.random.randint(4)      
        return 1
     
    def DQN(self):

        for episode in range(5): #this ain't working fix this. adding 5 for now
            state = self.reset()
            done = False
            while not done:
                print("Started")
                state_index = self.get_state_index(state)
                action = self.get_action(state_index)
                next_state, reward, done, _ = self.step(action)
                next_state_index = self.get_state_index(next_state)
                current_q_value = self.q_table[state_index][action]
                max_q_value = np.max(self.q_table[next_state_index])
                new_q_value = (1 - self.learning_rate) * current_q_value + self.learning_rate * (reward + self.discount_factor * max_q_value)
                self.q_table[state_index][action] = new_q_value
                state = next_state
            print(f"Episode: {episode + 1} completed.")
    
if __name__ == '__main__':
       env = GazeboEnv()
       while True:
            env.DQN()