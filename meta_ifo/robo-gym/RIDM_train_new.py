import os
import glob
import time
from datetime import datetime
import torch
import numpy as np
import gym
from IPython import embed
import robo_gym
from robo_gym.wrappers.exception_handling import ExceptionHandling
from PPO import PPO
import numpy as np
import torch.nn as nn
import copy
# from RIDM import RIDM, Inverse_dynamic_nn

from torch.autograd import Variable
import torch.nn.functional as F
import torch.optim as optim

import CMA_ES
from CMA_ES import CMA_ES



device = torch.device('cpu')






def get_expert_trajectory(env,env_name):

    print("================getting expert trajectory========================================================")    


    max_ep_len = 100          # max timesteps in one episode
    action_std = 0.1            # set same std for action distribution which was used while saving
    state_dim = env.observation_space.shape[0]
    action_dim = env.action_space.shape[0]
    has_continuous_action_space = True

    K_epochs = 80               # update policy for K epochs
    eps_clip = 0.2              # clip parameter for PPO
    gamma = 0.99                # discount factor

    lr_actor = 0.0003           # learning rate for actor
    lr_critic = 0.001           # learning rate for critic

    ppo_agent = PPO(state_dim, action_dim, lr_actor, lr_critic, gamma, K_epochs,
     eps_clip, has_continuous_action_space, action_std)

    random_seed = 0             #### set this to load a particular checkpoint trained on random seed
    run_num_pretrained = 0      #### set this to load a particular checkpoint num

    directory = "PPO_preTrained" + '/' + env_name + '/'
    checkpoint_path = directory + "PPO_{}_{}_{}.pth".format(env_name, random_seed, run_num_pretrained)
    print("loading network from : " + checkpoint_path)

    ppo_agent.load(checkpoint_path)

    final_reward = 0

    while final_reward != 100:
        trajectory = []
        trajectory_reward = []
        state, joint_positions ,ee_target_pose= env.reset_new()
        origin_state = copy.deepcopy(state)
        trajectory.append(state)

        output_reward = 0
        for i in range (max_ep_len):

            action = ppo_agent.select_action(state)
            print(np.max(action))
            action[action>1] = 1
            action[action<-1] = -1

            state, reward, done, _ = env.step(action)  
            print(reward)
            trajectory.append(state)
            trajectory_reward.append(reward)
            print("aaaaaaaaa",np.sum(origin_state - trajectory[0]))

            output_reward = output_reward + reward
            if done:
                final_reward = reward
                break      

    print(len(trajectory),len(trajectory_reward),output_reward)

    return trajectory,joint_positions ,ee_target_pose,output_reward






def RIDM_train(env, expert_trajectory,joint_positions ,ee_target_pose, train_times,output_reward):

    
    # state space dimension
    state_dim = env.observation_space.shape[0]

    # action space dimension
    action_dim = env.action_space.shape[0]
    total_number = 50
    sample_number = 15

    CMA_model = CMA_ES(30,total_number ,sample_number,env, expert_trajectory,
        joint_positions ,ee_target_pose)
    CMA_model.init()


    for iteration in range (train_times):

        CMA_model.sample_policy()
        CMA_model.act_policy()
        CMA_model.update()
        print("iteration",iteration,"avg reward",-CMA_model.period_reward/total_number,"expert reward",output_reward)

    mean = CMA_model.policy_mean
    cov = CMA_model.policy_cov





    # print(CMA_model.env.action_space)










def main():
    env_name = "ur3"
    target_machine_add = '127.0.0.1'
    env = gym.make('EndEffectorPositioningURSim-v0', ur_model=env_name, ip=target_machine_add)
    env = ExceptionHandling(env)    
    expert_trajectory,joint_positions ,ee_target_pose ,output_reward= get_expert_trajectory(env,env_name)
    has_continuous_action_space = True  # continuous action space; else discrete

    train_times = 10

    RIDM_train(env, expert_trajectory,joint_positions ,
        ee_target_pose,train_times,output_reward)







if __name__ == '__main__':

    main()
    
    
    
    
