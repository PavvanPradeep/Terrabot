#!/usr/bin/env python3
# license removed for brevity
import gym
import rospy
import numpy as np
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from gazebo_msgs.srv import SetModelState
from gazebo_msgs.msg import ModelState
from gym import spaces
from math import sqrt
from std_msgs.msg import Float64
import random
from stable_baselines3 import PPO
from gazebo_msgs.msg import ModelStates
import time 
import threading
import os 
import sys
# from ray.rllib.utils import check_env

class GazeboEnv(gym.Env):
    def __init__(self):
        rospy.init_node('terrabot_publisher', anonymous=True)
        # self.rate = rospy.Rate(10)
        self.model_state_msg = ModelState()
        self.model_state_msg.model_name = 'terrabot_design4_description'
        self.model_state_msg.pose.position.x = 0.0
        self.model_state_msg.pose.position.y = 0.0
        self.model_state_msg.pose.position.z = 0.0
        self.model_state_msg.pose.orientation.x = 0.0
        self.model_state_msg.pose.orientation.y = 0.0
        self.model_state_msg.pose.orientation.z = 0.0
        self.model_state_msg.pose.orientation.w = 1.0
        # self.observation_space = spaces.Box(low=np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0]), high=np.array([10.0, 10.0, 10.0, 10.0, 10.0, 10.0]), dtype=np.float32)
        # self.action_space = spaces.Box(low=np.array([-1.0, -1.0]), high=np.array([1.0, 1.0]), dtype=np.float32)
        self.rate = rospy.Rate(10)
        self.box_x = 0
        self.box_y = -1
        self.box_z = 0
        self.final_x = 0
        self.final_y = -7
        self.final_z = 0
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
        
        rospy.Subscriber("gazebo/model_states", ModelStates,self.update_position)
        self.reward_val = 0
        array_size = 1500000
        self.reward_array = [0] * array_size
        self.data = ModelStates()
        self.bot_name = "terrabot_design4_description"

  #SM's code
        self.observation_space = spaces.Box(low=np.array([0.0, 0.0, 0.0,0.0,0.0,0.0]), high=np.array([0.0, 0.0, 0.0,0.0,-10.0,0.0]), dtype=np.float32)
        self.Revolute_59_low = 0.3
        self.Revolute_59_high = 0.5
        self.Revolute_60_low =  -0.3
        self.Revolute_60_high = -0.5 
        self.Revolute_wheel_front_low = 10.0
        self.Revolute_wheel_front_high = 15.0
        self.Revolute_wheel_rear_low = 10.0
        self.Revolute_wheel_rear_high = 15.0
        low = np.array([self.Revolute_59_low, self.Revolute_60_low, self.Revolute_wheel_front_low, self.Revolute_wheel_rear_low])
        high = np.array([self.Revolute_59_high, self.Revolute_60_high, self.Revolute_wheel_front_high, self.Revolute_wheel_rear_high])
        self.action_space = spaces.Box(low=low, high=high, dtype=np.float32)
        self.continuous_thread = None
        self.continuous_event = threading.Event()
        
    def update_position(self, data):
        bot_name = "terrabot_design4_description"  # Replace this with the actual name of your robot model in Gazebo
        bot_index = data.name.index(bot_name)
        bot_pose = data.pose[bot_index]
        self.latest_position = bot_pose.position
        
        x=self.latest_position.x
        x=round(x,3)
        y=self.latest_position.y
        y=round(y,3)
        z=self.latest_position.z
        z=round(z,3)
        self.a=[x,y,z]
        print(self.a)
        self.calculate_reward()
        
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

    def change_angle(self,pub,pub1):
        self.rate = rospy.Rate(10)
        new_pos_f=random.uniform(-0.5,0.3)
        new_pos_r=random.uniform(0.3,0.5)
        pub.publish(new_pos_r)
        pub1.publish(new_pos_f)
        
    
    def change_velocity(self,pub3,pub2):
        self.rate = rospy.Rate(10)
        self.data = Twist()
        self.data.linear.x = 0.0
        self.data.linear.y = 0.0
        self.data.linear.z = 0.0
        self.data.angular.x = 0.0
        self.data.angular.y = 0.0
        # self.data.angular.z = random.uniform(10,15)
        # self.new_velocity = self.data.angular.z
        self.new_velocity = random.uniform(10, 15)  
        self.data.angular.z = self.new_velocity
        print("changed the velocity",self.data.angular.z)
        rospy.loginfo(self.data)
        pub3.publish(self.data)
        pub2.publish(self.data)


    def Revolute_wheel_front(self, pub2):
        self.rate = rospy.Rate(10)
        data1 = Twist()
        data1.linear.x = 0.0
        data1.linear.y = 0.0
        data1.linear.z = 0.0
        data1.angular.x = 0.0
        data1.angular.y = 0.0
        data1.angular.z = 10.0
        rospy.loginfo(data1)
        pub2.publish(data1)
    
    def Revolute_wheel_rear(self, pub3):
        self.rate = rospy.Rate(10)
        data2 = Twist()
        data2.linear.x = 0.0
        data2.linear.y = 0.0
        data2.linear.z = 0.0
        data2.angular.x = 0.0
        data2.angular.y = 0.0
        data2.angular.z = 10.0
        rospy.loginfo(data2)
        pub3.publish(data2)
        
    def Stop_front(self,pub2):
        data3=Twist()
        data3.linear.x = 0.0
        data3.linear.y = 0.0
        data3.linear.z = 0.0
        data3.angular.x = 0.0
        data3.angular.y = 0.0
        data3.angular.z = 10.0
        # rospy.loginfo(data3)
        pub2.publish(data3)

    def Stop_rear(self,pub3):
        data3=Twist()
        data3.linear.x = 0.0
        data3.linear.y = 0.0
        data3.linear.z = 0.0
        data3.angular.x = 0.0
        data3.angular.y = 0.0
        data3.angular.z = 0.0
        # rospy.loginfo(data3)
        pub3.publish(data3)
       
    def reset(self):
        self.set_model_state_proxy = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
        self.set_model_state_msg = ModelState()
        self.set_model_state_msg.model_name = 'terrabot_design4_description'
        self.set_model_state_msg.pose.position.x = 0
        self.set_model_state_msg.pose.position.y = 0
        self.set_model_state_msg.pose.position.z = 0
        self.set_model_state_msg.pose.orientation.x = 0
        self.set_model_state_msg.pose.orientation.y = 0
        self.set_model_state_msg.pose.orientation.z = 0
        self.set_model_state_msg.pose.orientation.w = 1
        self.set_model_state_proxy(self.set_model_state_msg)
        print("in reset function") 
        return self.get_obs()

    def get_obs(self):
        if self.latest_position is None:
            return [self.box_x, self.box_y, self.box_z, 0, 0, 0]
        else:
            return [self.box_x, self.box_y, self.box_z, round(self.latest_position.x,3),round(self.latest_position.y,3), round(self.latest_position.z,3)]
        
    def step(self,action):
        
        self.action=action
        print("action",action)
        self.start_continuous_thread(action)
        continuous_thread = threading.Thread(target= env.run_continuous,args=(action,))
        continuous_thread.daemon = True  # Set the thread as a daemon, it will terminate when the main thread ends
        continuous_thread.start() 
        # self.move_robot(action)
        obs = self.get_obs()
        print("this is the obs recieved",obs)
        print("this is the prev reward",self.reward_array[-2])
        print("this is the one which is 4 steps behind",self.reward_array[-4])
        reward = self.reward()
        if self.reward_array[-2]==reward:
            self.change_velocity(self.pub3,self.pub2)
            # self.change_angle(self.pub,self.pub1)
        done=self.done(self.position_x,self.box_x,self.position_y,self.box_y)
        print("completed step function")
        self.stop_continuous_thread()
        return obs,reward,done,{}
    
    def calculate_reward(self):

        # if self.latest_position is not None:
        x = self.a[0]
        y = self.a[1]
        #z = self.latest_position.z
        self.distance = sqrt((x - self.box_x) ** 2 + (y - self.box_y) ** 2)
        self.distance=round(self.distance,3)
        self.reward_val = -self.distance
        self.reward_val=round(self.reward_val,3)
        self.reward_array.append(self.reward_val)
        print("Reward obtained:", self.reward_val)
        
    def reward(self):
        return self.reward_val
    
    def prev(self):
        prev_reward=self.reward()
        return prev_reward
    
    def move_robot(self,action):
        # if action == 1:
        #     self.actions_array.append(self.Revolute_59(self.pub))
        #     self.actions_array.append(self.Revolute_60(self.pub1))
        #     self.actions_array.append(self.Revolute_wheel_front(self.pub2))
        #     self.actions_array.append(self.Revolute_wheel_rear(self.pub3))
        # elif action == 0:
        #     self.actions_array.append(self.Stop_front(self.pub2))
        #     self.actions_array.append(self.Stop_rear(self.pub3))
        # return self.actions_array

#SM's code :
        
        front_wheel_action = np.clip(action[2], self.Revolute_wheel_front_low, self.Revolute_wheel_front_high)
        rear_wheel_action = np.clip(action[3], self.Revolute_wheel_rear_low, self.Revolute_wheel_rear_high)
        self.actions_array.append(self.Revolute_59(self.pub))
        self.actions_array.append(self.Revolute_60(self.pub1))
        data1 = Twist()
        data1.angular.z = front_wheel_action
        
        # self.data1.angular.z = front_wheel_action
        # data1 = self.data.angular.z
        # data1 = self.new_velocity
       
        self.pub2.publish(data1)
        time.sleep(0.2)
        data2 = Twist()
        # self.data1.angular.z = rear_wheel_action
        # data2 = self.data.angular.z
        # data2 = self.new_velocity
        data2.angular.z = rear_wheel_action

        self.pub3.publish(data2)
        time.sleep(0.2)
        self.rate.sleep()
 
    def done(self,x,y,final_x,final_y):
        x = self.a[0]
        y = self.a[1]
        self.distance = sqrt((x - self.final_x) ** 2 + (y - self.final_y) ** 2)
        print("in done function here is the distance",self.distance)
        if self.distance==0:
            return True
        # elif (self.reward_val)==(self.reward_array[-10]):#toppling work nahi kar raha hein
        #     return True
        elif self.reward_val <= -10:
            return True
        else: return False

    def run_continuous(self,actions):
        rate = rospy.Rate(10)  # Adjust the rate as needed
        while not rospy.is_shutdown() and self.continuous_event.is_set():
            # Call the move_robot function here with the desired action (1 in this case)
            #print(action)
            self.move_robot(actions)
            rate.sleep()
    
    def start_continuous_thread(self,actions):
        self.continuous_event.set()  
        continuous_thread = threading.Thread(target= env.run_continuous,args=(actions,))
        continuous_thread.daemon = True  # Set the thread as a daemon, it will terminate when the main thread ends
        continuous_thread.start()

    def stop_continuous_thread(self):
        self.continuous_event.clear() 
        if self.continuous_thread:
            self.continuous_thread.join()
            self.continuous_thread = None
        
if __name__ == '__main__':
       env = GazeboEnv()
       model = PPO('MlpPolicy', env, verbose=1)
       model.learn(total_timesteps=1000)
       save_path = os.path.join('/home/srimitravinda/catkin_ws/src/Terrabot__/terrabot_design4_description/src/envs')
       model.save(save_path)
    #    model = PPO.load("envs")
       obs = env.reset()
       for i in range(50):
            action, _state = model.predict(obs, deterministic=True)
            obs, reward, done, info = env.step(action)
            if done==True:
                obs = env.reset()