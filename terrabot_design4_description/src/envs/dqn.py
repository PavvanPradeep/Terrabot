# from torch import nn
# import torch
# import gym
# from collections import deque
# import itertools
# import numpy as np
# import random

# GAMMA = 0.99  #discount rate for computing temporal difference target
# BATCH_SIZE = 32 #transition to sample from replay buffer when computing gradients
# BUFFER_SIZE = 50000 #maximum number of transitions we sample from replay buffer
# MIN_REPLAY_SIZE = 1000 #how many transition we want in replay buffer before we start training
# EPSILON_START = 1.0 #start value
# EPSILON_END = 0.02 #end value
# EPSILON_DECAY = 10000 #decay period - epsilon value will linearly reduce
# TARGET_UPDATE_FREQ = 1000 #set target params to online params ?

# class Network(nn.Module):
#     def __init__(self, env):
#         super().__init__()
#         in_features = int(np.prod(env.observation_space.shape))

#         self.net = nn.Sequential(
#             nn.Linear(in_features,65),
#             nn.Tanh(),
#             nn.Linear(64, env.action_space.n)
#         )
#     def forward(self,x):
#         return self.net(x)
    
#     def act(self,obs):
#         pass

# replay_buffer = deque(maxlen=BUFFER_SIZE)

# row_buffer = deque([0,0],maxlen=100)
# episode_reward = 0.0

# online_net = Network(env)
# target_net = Network(env)

# target_net.load_state_dict(online_net.state_dict())

# obs = env.reset()
# for _ in range(MIN_REPLAY_SIZE):
#     action = env.

