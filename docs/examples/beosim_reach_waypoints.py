import gym, robo_gym
import numpy as np
from robo_gym.wrappers.taskEnv import MoveEffectorToWayPoints

env = gym.make('BeoarmTestBlock-v0', gui=True, ip='127.0.0.1')
wayPoints = np.array([
		[0.174, 0, 0.58],
		[-0.174, 0, 0.677]
		])
env = MoveEffectorToWayPoints(env, wayPoints, 'ee_to_ref_translation', distanceThreshold=0.04)
done = False
while not done:
    state, reward, done, info = env.step([0, 0, 1, 0.8])
    if 'atWaypoint' in info and info['atWaypoint'] != []:
    	print('reach wayPoint:', info['atWaypoint'])
    if done:
    	print('End-effector reached all wayPoints!')
