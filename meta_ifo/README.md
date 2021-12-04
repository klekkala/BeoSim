# meta_ifo
This is a repo forked from [https://github.com/klekkala/meta_ifo](https://github.com/klekkala/meta_ifo).

In meta_ifo/robo-gym directory, it contains an extra file named train_beosim.py that uses deep reinforcement learning to train the beosim to reach the goal. See this [paper](https://arxiv.org/abs/1906.07372) for details.

If you install robo-gym correctly, you should be able to run this file and train the robot to push the cube to the destination.

The model is imperfect, as it doesn't seem to learn anything even though we run it for days. Some notes about this file:
1. The model does not have a expert demonstrator, so it takes a long time to explore the environment.

2. The reward is based on the following scheme:
- Small reward (max 1) for the end-effector getting close to the cube.
- Small penalty for the end-effector getting far away from the cube.
- Moderate reward (max 20) for the cube geeting close to the target.
- +300 reward if the cube reaches the target.
- -100 reward if the robot collides anything other than the cube.

3. During training, it will save the model in PPO_preTrained/beosim folder
