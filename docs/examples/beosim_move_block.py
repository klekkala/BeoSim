"""
Before running this file, please run 'start-server-manager' first.
"""
import gym, robo_gym
import numpy as np
from robo_gym.wrappers.taskEnv import MoveObjectToTargetTask

env = gym.make('BeoarmTestBlock-v0', gui=True, ip='127.0.0.1')
startingPosition = [0, 0.3, 1]
targetPosition = np.array([-0.24, 0.17, 0])
env.reset(startingPosition)
env = MoveObjectToTargetTask(env, 'wood_cube_10cm', targetPosition, distanceThreshold=0.04)

done = False
for i in range(60):
    env.step([0, 0, 1, 0.8])
for i in range(28):
    env.step([0, 1, 0, 0])
while not done:
    _, _, done, _ = env.step([1, 0.2, 0, 0])

print('object reach the target position!')
