from gym.envs.registration import register

register(id = 'Robot-v0', 
         entry_point = 'envs.robot_envs.terrabot_env:RobotEnv',
         max_episode_steps = 1000,)