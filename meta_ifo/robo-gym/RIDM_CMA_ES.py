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



import cma


device = torch.device('cpu')



def get_expert_trajectory(env,env_name):

    print("================getting expert trajectory========================================================")    


    max_ep_len = 60          # max timesteps in one episode
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
        out_reward = 0
        for i in range (max_ep_len):

            action = ppo_agent.select_action(state)
            print(np.max(action))
            action[action>1] = 1
            action[action<-1] = -1

            state, reward, done, _ = env.step(action)  
            out_reward = out_reward + reward
            print(reward)
            trajectory.append(state)
            trajectory_reward.append(reward)
            print("aaaaaaaaa",np.sum(origin_state - trajectory[0]))
            if done:
                final_reward = reward
                break      

    print(len(trajectory),len(trajectory_reward))

    return trajectory,joint_positions ,ee_target_pose,out_reward


def compute_weight_decay(weight_decay, model_param_list):
    model_param_grid = np.array(model_param_list)
    return - weight_decay * np.mean(model_param_grid * model_param_grid, axis=1)

class CMAES:
    '''CMA-ES wrapper.'''
    def __init__(self, num_params,      # number of model parameters
               sigma_init=0.10,       # initial standard deviation
               popsize=255,           # population size
               weight_decay=0.01):    # weight decay coefficient

        self.num_params = num_params
        self.sigma_init = sigma_init
        self.popsize = popsize
        self.weight_decay = weight_decay
        self.solutions = None

        
        self.es = cma.CMAEvolutionStrategy( self.num_params * [0],
                                            self.sigma_init,
                                            {'popsize': self.popsize,
                                            })

    def rms_stdev(self):
        sigma = self.es.result[6]
        return np.mean(np.sqrt(sigma*sigma))

    def ask(self):
        '''returns a list of parameters'''
        self.solutions = np.array(self.es.ask())
        return self.solutions

    def tell(self, reward_table_result):
        reward_table = -np.array(reward_table_result)
        if self.weight_decay > 0:
            l2_decay = compute_weight_decay(self.weight_decay, self.solutions)
            reward_table += l2_decay
        self.es.tell(self.solutions, (reward_table).tolist()) # convert minimizer to maximizer.

    def current_param(self):
        return self.es.result[5] # mean solution, presumably better with noise

    def set_mu(self, mu):
        pass

    def best_param(self):
        return self.es.result[0] # best evaluated solution

    def result(self): # return best params so far, along with historically best reward, curr reward, sigma
        r = self.es.result
        return (r[0], -r[1], -r[1], r[6])

def evaluate(env,solution,expert_trajectory,joint_positions,ee_target_pose):

    solution_1 = solution[0:450]
    solution_2 = solution[450:]

    solution_1  = solution_1.reshape([30,15])
    solution_2  = solution_2.reshape([15,5])

    state = env.reset_2(joint_positions,ee_target_pose)

    out_reward = 0

    for i in range (1,len(expert_trajectory)):

        expert_state = expert_trajectory[i]

        input_state = np.concatenate([expert_state,state]).reshape([1,-1])

        mid_1 = np.matmul(input_state,solution_1) 

        mid_1 = np.tanh(mid_1)

        action = np.squeeze(np.matmul(mid_1,solution_2))

        action = np.tanh(action)

        state, reward, done, _ = env.step(action)
        out_reward = out_reward + reward

        if done:
            break

    return out_reward

        


def main(step):


    env_name = "ur3"
    target_machine_add = '127.0.0.1'
    env = gym.make('EndEffectorPositioningURSim-v0', ur_model=env_name, ip=target_machine_add)
    env = ExceptionHandling(env)    
    expert_trajectory,joint_positions ,ee_target_pose,expert_reward = get_expert_trajectory(env,env_name)

    num_params = 525  # number of model parameters
    sigma_init = 0.50 # initial standard deviation
    popsize = 255      # population size
    weight_decay=0.01

    

    solver = CMAES(num_params, sigma_init, popsize, weight_decay)

    for p in range (step):
        solutions = solver.ask()
        rewards = np.zeros(solver.popsize)

        for i in range(solver.popsize):
            rewards[i] = evaluate(env,solutions[i],expert_trajectory,
                joint_positions,ee_target_pose)
        print("iteration",p,"mean reward",np.mean(rewards),"expert_reward",expert_reward)

        solver.tell(rewards)

        reward_vector = solver.result()
        print("current reward",reward_vector[1])




if __name__ == '__main__':

    step = 20
    main(step)
    
    
    
    
