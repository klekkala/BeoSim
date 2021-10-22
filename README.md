# robo-gym 

![PyPI - Python Version](https://img.shields.io/pypi/pyversions/robo-gym)
![PyPI](https://img.shields.io/pypi/v/robo-gym)

This is a repo forked from the original [robo-gym](https://github.com/jr-robotics/robo-gym) by jr-robotics. I added a customized robot model and few environments on top of robo-gym for our lab.

# Installation
1. Install [modified robo-gym-robot-servers](https://github.com/tinhangchui/robo-gym-robot-servers).

2. Install [modified robo-gym-server-modules](https://github.com/tinhangchui/robo-gym-server-modules).

3. Finally, run the following commands:
```bash
git clone https://github.com/tinhangchui/robo-gym.git
cd robo-gym
pip install -e .
```

# New features

### MoveObjectToTargetTask Wrapper
This wrapper adds a goal of moving an object to a given destination. The training is considered done only when the object is close enough to the destination.

Example code:
```
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
```
In this example, we want to move a block, which is located at (0, 0.3, 1) at the beginning, to the destination position of (-0.24, 0.17, 0).

The line
```
env = MoveObjectToTargetTask(env, 'wood_cube_10cm', targetPosition, distanceThreshold=0.04)
```
uses MoveObjectToTargetTask to wrap the environment such that it has a goal of moving wood_cube_10cm to targetPosition. The variable 'done' is set to true when the distance between wood_cube_10cm and the targetPosition is less or equal to  0.04.

If not specified, distanceThreshold is 0.3 by default.

### MoveEffectorToWayPoints Wrapper
This wrapper adds a goal of moving the end-effector to visit each given waypoints at least once.

Example code:
```
import gym, robo_gym
import numpy as np
from robo_gym.wrappers.taskEnv import MoveEffectorToWayPoints

env = gym.make('BeoarmTestBlock-v0', gui=True, ip='127.0.0.1')
wayPoints = np.array([
		[0.174, 0, 0.58],
		[-0.174, 0, 0.677]
		])
env.reset(startingPosition)
env = MoveEffectorToWayPoints(env, wayPoints, 'ee_to_ref_translation', distanceThreshold=0.04)
done = False
while not done:
    state, reward, done, info = env.step([0, 0, 1, 0.8])
    if 'atWaypoint' in info and info['atWaypoint'] != []:
    	print('reach wayPoint:', info['atWaypoint'])
    if done:
    	print('End-effector reached all wayPoints!')
```
In this example, we want to let the end-effector visits (0.174, 0, 0.58) and (-0.174, 0, 0.677).

The line
```
env = MoveEffectorToWayPoints(env, wayPoints, 'ee_to_ref_translation', distanceThreshold=0.04)
```
uses MoveEffectorToWayPoints to wrap the environment such that the end-effector must reach 'wayPoints' of (0.174, 0, 0.58) and (-0.174, 0, 0.677), and the name of the end-effector object is 'ee_to_ref_translation'. 

When the distance between the end-effector and a waypoint is less than or equal to 0.04, it is considered that the end-effector has reached the waypoint. 

The variable 'done' is set to true when the end-effector has reached all waypoints.

### Reward
When the goal is completed, the environment gives reward = 1. Otherwise, it will give reward = -0.01.

One can change the reward by deriving from MoveObjectToTargetTask or MoveEffectorToWayPoints, and overwrite the reward function.
