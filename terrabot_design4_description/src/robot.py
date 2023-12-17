import gym
import envs

env = gym.make("Robot-v0",

            )
for i in range(0,4):
    obs,info = env.reset()
    done=False
    while not done:
        env.render()
        action=env.action_space.sample()
        obs, reward, terminated, truncated, info = env.step(action)
        print('Obs:{},reward:{},terminated:{},truncated:{},info:{}'.format(obs,reward,terminated,truncated,info))
        
env.close()