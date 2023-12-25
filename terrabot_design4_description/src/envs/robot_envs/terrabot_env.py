import gym
import numpy
import rospy
import time
from envs import robot_gazebo_env
from gazebo_msgs.msg import ContactsState
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Quaternion, Vector3
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist

# class RobotEnv(gym.Env):
#     def __init__(self):
#         ...

class TerrabotEnv(robot_gazebo_env.RobotGazeboEnv):
    def __init__(self):
        rospy.logdebug("Start Terrabot Env INIT...")

        self.controllers_list = ['Revolute_59_position_controller','Revolute_60_position_controller','mobile_base_controller','mobile_base_controller1']

        self.robot_name_space = "/"

        super(TerrabotEnv,self).__init__(controllers_list = self.controllers_list,
                                         robot_name_space = self.robot_name_space,
                                         reset_controls = False,
                                         start_init_physics_parameters = False,
                                         reset_world_or_sim = "WORLD")
        rospy.logdebug("TerrabotEnv unpause1...")
        # self.gazebo.unpauseSim()

        self.check_all_systems_ready()

          # We Start all the ROS related Subscribers and publishers
        rospy.Subscriber("/Revolute_59_position_controller/command", Float64, self._Revolute59_callback)
        # We use the IMU for orientation and linearacceleration detection
        rospy.Subscriber("/Revolute_60_position_controller/command", Float64, self._Revolute60_callback)
        # We use it to get the contact force, to know if its in the air or stumping too hard.
        rospy.Subscriber("/mobile_base_controller/cmd_vel", Twist, self._wheels1_callback)
        rospy.Subscriber("/mobile_base_controller/odom", Odometry, self._wheels1_odom_callback)
        # We use it to get the joints positions and calculate the reward associated to it
        rospy.Subscriber("/mobile_base_controller1/cmd_vel", Twist, self._wheels2_callback)
        rospy.Subscriber("/mobile_base_controller1/odom", Odometry, self._wheels2_odom_callback)


        self.publishers_array = []

        self.pub_joint1 = rospy.Publisher('/Revolute_59_position_controller/command',
						Float64, queue_size=10)
        self.pub_joint2 = rospy.Publisher('/Revolute_60_position_controller/command',
						Float64, queue_size=10)
        self.pub_wheel1 = rospy.Publisher('/mobile_base_controller/cmd_vel',
						Twist, queue_size=10)
        self.pub_wheel2 = rospy.Publisher('/mobile_base_controller1/cmd_vel',
						Twist, queue_size=10)
        
        self.publishers_array.append(self.pub_joint1)
        self.publishers_array.append(self.pub_joint2)
        self.publishers_array.append(self.pub_wheel1)
        self.publishers_array.append(self.pub_wheel2)

        # self._check_all_publishers_ready()

        # self.gazebo.pauseSim()
        
        rospy.logdebug("Finished HopperEnv INIT...")



    def _check_all_systems_ready(self):
        """
        Checks that all the sensors, publishers and other simulation systems are
        operational.
        """
        rospy.logdebug("HopperEnv check_all_systems_ready...")
        self._check_all_sensors_ready()
        rospy.logdebug("END HopperEnv _check_all_systems_ready...")
        return True

    def _check_joint_states_ready(self):
        self.joint_states = None
        rospy.logdebug("Waiting for /joint_states to be READY...")
        while self.joint_states is None and not rospy.is_shutdown():
            try:
                self.joint_states = rospy.wait_for_message("/monoped/joint_states", JointState, timeout=1.0)
                rospy.logdebug("Current /joint_states READY=>")

            except:
                rospy.logerr("Current /joint_states not ready yet, retrying for getting joint_states")
        return self.joint_states
    
    def _joint_state_callback(self,data):
        self.joint_states = data
    
    
    def _check_all_publishers_ready(self):
        """
        Checks that all the publishers are working
        :return:
        """
        # rospy.logdebug("START ALL SENSORS READY")
        for publisher_object in self.publishers_array:
            self._check_pub_connection(publisher_object)
        rospy.logdebug("ALL SENSORS READY")
    

    def _check_pub_connection(self, publisher_object):

        rate = rospy.Rate(10)  # 10hz
        while publisher_object.get_num_connections() == 0 and not rospy.is_shutdown():
            rospy.logdebug("No susbribers to publisher_object yet so we wait and try again")
            try:
                rate.sleep()
            except rospy.ROSInterruptException:
                # This is to avoid error when world is rested, time when backwards.
                pass
        rospy.logdebug("publisher_object Publisher Connected")

        rospy.logdebug("All Publishers READY")
    
    def _set_init_pose(self):
        """Sets the Robot in its init pose
        """
        raise NotImplementedError()
    
    def _init_env_variables(self):
        """Inits variables needed to be initialised each time we reset at the start
        of an episode.
        """
        raise NotImplementedError()

    def _compute_reward(self, observations, done):
        """Calculates the reward to give based on the observations given.
        """
        raise NotImplementedError()

    def _set_action(self, action):
        """Applies the given action to the simulation.
        """
        raise NotImplementedError()

    def _get_obs(self):
        raise NotImplementedError()

    def _is_done(self, observations):
        """Checks if episode done based on observations given.
        """
        raise NotImplementedError()
    
    def move_joints(self, joints_array, epsilon=0.05, update_rate=10, time_sleep=0.05, check_position=True):
        """
        It will move the Hopper Joints to the given Joint_Array values
        """
        i = 0
        for publisher_object in self.publishers_array:
          joint_value = Float64()
          wheel_value = Twist()

          joint_value.data = joints_array[i]
          rospy.logdebug("JointsPos>>"+str(joint_value))
          publisher_object.publish(joint_value)
          i += 1
        
        if check_position:
            self.wait_time_for_execute_movement(joints_array, epsilon, update_rate)
        else:
            self.wait_time_movement_hard(time_sleep=time_sleep)
      

    def wait_time_for_execute_movement(self, joints_array, epsilon, update_rate):
        """
        We wait until Joints are where we asked them to be based on the joints_states
        :param joints_array:Joints Values in radians of each of the three joints of hopper leg.
        :param epsilon: Error acceptable in odometry readings.
        :param update_rate: Rate at which we check the joint_states.
        :return:
        """
        rospy.logdebug("START wait_until_twist_achieved...")
        
        rate = rospy.Rate(update_rate)
        start_wait_time = rospy.get_rostime().to_sec()
        end_wait_time = 0.0

        rospy.logdebug("Desired JointsState>>" + str(joints_array))
        rospy.logdebug("epsilon>>" + str(epsilon))
        
        while not rospy.is_shutdown():
            current_joint_states = self._check_joint_states_ready()
            
            values_to_check = [ current_joint_states.position[0],
                                current_joint_states.position[1],
                                current_joint_states.position[2]]
            
            vel_values_are_close = self.check_array_similar(joints_array,values_to_check,epsilon)
            
            if vel_values_are_close:
                rospy.logdebug("Reached JointStates!")
                end_wait_time = rospy.get_rostime().to_sec()
                break
            rospy.logdebug("Not there yet, keep waiting...")
            rate.sleep()
        delta_time = end_wait_time- start_wait_time
        rospy.logdebug("[Wait Time=" + str(delta_time)+"]")
        
        rospy.logdebug("END wait_until_jointstate_achieved...")
        
        return delta_time

  