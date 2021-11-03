#!/usr/bin/env python3
import copy
import numpy as np
import gym
from random import uniform, choice
from typing import Tuple
from robo_gym.utils.exceptions import InvalidStateError, RobotServerError, InvalidActionError
import robo_gym_server_modules.robot_server.client as rs_client
from robo_gym_server_modules.robot_server.grpc_msgs.python import robot_server_pb2
from robo_gym.envs.simulation_wrapper import Simulation

joint_names = ['joint_1', 'joint_2', 'joint_3', 'joint_4']
DISTANCE_THRESHOLD = 0.1

class BeoarmEEPosition(gym.Env, Simulation):
    def __init__(self, ip='127.0.0.1', lower_bound_port=None, upper_bound_port=None, gui=False, **kwargs):
        self.cmd = 'roslaunch beoarm_robot_server robot_server.launch \
                    world_name:=test.world \
                    action_cycle_rate:=20 \
                    rviz_gui:=false \
                    gazebo_gui:=true \
                    object_model_names:=[box100]'
        Simulation.__init__(self, self.cmd, ip, lower_bound_port, upper_bound_port, gui, **kwargs)
        # self.robot_server_ip is generated from Simulation after calling _start_sim()
        rs_address = self.robot_server_ip

        self.elapsed_steps = 0
        self.max_episode_steps = 300
        self.abs_joint_pos_range = [6.28] * 4

        # state contains only joint positions scaled to [-1, 1]
        self.state = [0.0] * len(joint_names)
        # state_dict contains joint positions, velocities, transitions from ee and object_0 to reference frame, and collision
        self.state_dict = dict.fromkeys(self._get_robot_server_composition(), 0.0)

        # Connect to Robot Server
        if rs_address:
            self.client = rs_client.Client(rs_address)
        else:
            print('WARNING: No IP and Port passed. Simulation will not be started')
            print('WARNING: Use this only to get environment shape')


    def reset(self, target_object_position=None) -> np.array:
        # calls ros_bridge's reset_robot_object function with new target object position
        if target_object_position:
            if not self._check_valid_ee_trans(target_object_position):
                raise InvalidStateError('Invalid target_object_position {}'.format(target_object_position))
        else:
            target_object_position = self._get_random_valid_ee_trans()

        reset_msg = robot_server_pb2.State(
            float_params={
                'object_0_x': target_object_position[0],
                'object_0_y': target_object_position[1],
                'object_0_z': target_object_position[2]
            }
        )

        # FIXME: add Exception for reset?
        if not self.client.reset_state_msg(reset_msg):
            raise RobotServerError('set_state')

        self.elapsed_steps = 0

        # retrieves state from ros_bridge after reset
        new_state_dict = self.client.get_state_msg().state_dict
        self._check_state_dict_keys(new_state_dict)
        normalized_joint_positions = self._normalize_joint_positions([new_state_dict[j+'_position'] for j in joint_names])
        self._check_valid_action(normalized_joint_positions)
        self.state = normalized_joint_positions
        self.state_dict = new_state_dict

        return np.array(normalized_joint_positions)


    '''
    resets joint positions to specified values by passing them as strict actions,
        thus allowing resetting to joint_positions that is not all-zeros;
        otherwise the same as reset

    (might encounter collision problems)
    '''
    def reset_soft(self, target_joint_positions, target_object_position=None):
        if target_object_position:
            if not self._check_valid_ee_trans(target_object_position):
                raise InvalidStateError('Invalid target_object_position {}'.format(target_object_position))
        else:
            target_object_position = self._get_random_valid_ee_trans()

        set_msg = robot_server_pb2.State(
            state_dict = dict(zip(joint_names, target_joint_positions)),
            float_params={
                'object_0_x': target_object_position[0],
                'object_0_y': target_object_position[1],
                'object_0_z': target_object_position[2]
            }
        )

        if not self.client.set_state_msg(set_msg):
            raise RobotServerError('set_state')

        self.elapsed_steps = 0

        new_state_dict = self.client.get_state_msg().state_dict
        self._check_state_dict_keys(new_state_dict)
        normalized_joint_positions = self._normalize_joint_positions([new_state_dict[j+'_position'] for j in joint_names])
        self._check_valid_action(normalized_joint_positions)
        if not np.isclose(normalized_joint_positions, target_joint_positions, atol=0.05).all():
            raise InvalidStateError('Reset joint positions are not within defined range')

        self.state = normalized_joint_positions
        self.state_dict = new_state_dict

        return np.array(normalized_joint_positions)


    def reward(self, state_dict, action) -> Tuple[float, bool, dict]:
        reward = 0
        done = False
        info = {}

        # Calculate distance to the target
        target_coord = np.array([state_dict['object_0_to_ref_translation_x'], state_dict['object_0_to_ref_translation_y'], state_dict['object_0_to_ref_translation_z']])
        ee_coord = np.array([state_dict['ee_to_ref_translation_x'], state_dict['ee_to_ref_translation_y'], state_dict['ee_to_ref_translation_z']])
        euclidean_dist_3d = np.linalg.norm(target_coord - ee_coord)

        # Reward base
        reward = -1 * euclidean_dist_3d

        # reward = reward + (-1/300)

        # Joint positions
        joint_positions_keys = [j+'_position' for j in joint_names]
        joint_positions = [state_dict[p] for p in joint_positions_keys]
        joint_positions_normalized = self._normalize_joint_positions(joint_positions)

        # FIXME
        delta = np.abs(np.subtract(joint_positions_normalized, action))
        # reward = reward - (0.05 * np.sum(delta))

        if euclidean_dist_3d <= DISTANCE_THRESHOLD:
            reward = 100
            done = True
            info['final_status'] = 'success'
            info['target_coord'] = target_coord

        # Check if robot is in collision
        if state_dict['in_collision'] == 1:
            collision = True
        else:
            collision = False

        if collision:
            reward = -400
            done = True
            info['final_status'] = 'collision'
            info['target_coord'] = target_coord

        if self.elapsed_steps >= self.max_episode_steps:
            done = True
            info['final_status'] = 'max_steps_exceeded'
            info['target_coord'] = target_coord

        return reward, done, info


    '''
    guarantee: if true then guarantees each action is performed to an accuracy threshold
    '''
    def step(self, action, guarantee=False) -> Tuple[np.array, float, bool, dict]:
        self.elapsed_steps += 1

        self._check_valid_action(action)
        rs_action = self._scale_action(action)

        if guarantee:
            new_state_dict = self.client.send_strict_action_get_state(rs_action).state_dict
        else:
            new_state_dict = self.client.send_action_get_state(rs_action).state_dict

        self._check_state_dict_keys(new_state_dict)
        normalized_joint_positions = self._normalize_joint_positions([new_state_dict[j+'_position'] for j in joint_names])
        self._check_valid_action(normalized_joint_positions)

        self.state = normalized_joint_positions
        self.state_dict = new_state_dict

        # Assign reward
        reward = 0
        done = False
        reward, done, info = self.reward(state_dict=new_state_dict, action=action)
        info['state_dict'] = new_state_dict

        return np.array(normalized_joint_positions), reward, done, info


    '''
    goal: [position_x, position_y, position_z]
        default to [object_0_x, object_0_y, object_0_z]

    returns False if plan failed
        otherwise returns list of position lists
    '''
    def plan(self, goal=None):
        if goal:
            if not self._check_valid_ee_trans(goal):
                raise InvalidStateError('Invalid goal {}'.format(goal))
        else:
            goal = [self.state_dict['object_0_to_ref_translation_x'],
                    self.state_dict['object_0_to_ref_translation_y'],
                    self.state_dict['object_0_to_ref_translation_z']]

        plan = self.client.send_goal_get_plan(goal)
        if plan.success:
            # allow numerical errors when comparing requested and received goals
            max_diff = max([abs(a-b) for a, b in zip(plan.goal.pose, goal)])
            if max_diff <= 0.1:
                commands = []
                for pt in plan.trajectory:
                    prop_jp = self._normalize_joint_positions(pt.positions)
                    commands.append(prop_jp)
                return commands
            else:
                raise Exception('Retrieved plan for another goal ' + str(plan.goal.pose) + ' , whereas requested goal is ' + str(goal))
        else:
            return False


    '''
    plan: list of position lists
    skip_to_end: if true then directly sets joint positions to the IK solution of goal;
                    speeds up execution but only use when no collision;
                    otherwise visits each trajectory point
    '''
    def execute_plan(self, plan, skip_to_end=False):
        if not skip_to_end:
            for pt in plan[:-1]:
                self.step(pt, guarantee=True)

        return self.step(plan[-1], guarantee=True)


    '''
    check if action is in range [-1, 1]
    '''
    def _check_valid_action(self, action):
        if not all([-1 <= a <= 1 for a in action]):
            raise InvalidActionError()


    '''
    scale action from [-1, 1] to [min_joint_position, max_joint_position]
    '''
    def _scale_action(self, action):
        return [a*r for a,r in zip(action, self.abs_joint_pos_range)]


    '''
    scale from [min_joint_position, max_joint_position] to [-1, 1]
    '''
    def _normalize_joint_positions(self, joint_positions):
        return [p/float(r) for p,r in zip(joint_positions, self.abs_joint_pos_range)]


    '''
    what state_dict should contain
    '''
    def _get_robot_server_composition(self) -> list:
        state_dict_keys = [
            'joint_1_position',
            'joint_2_position',
            'joint_3_position',
            'joint_4_position',

            'joint_1_velocity',
            'joint_2_velocity',
            'joint_3_velocity',
            'joint_4_velocity',

            'ee_to_ref_translation_x',
            'ee_to_ref_translation_y',
            'ee_to_ref_translation_z',
            'ee_to_ref_rotation_x',
            'ee_to_ref_rotation_y',
            'ee_to_ref_rotation_z',
            'ee_to_ref_rotation_w',

            'object_0_to_ref_translation_x',
            'object_0_to_ref_translation_y',
            'object_0_to_ref_translation_z',
            'object_0_to_ref_rotation_x',
            'object_0_to_ref_rotation_y',
            'object_0_to_ref_rotation_z',
            'object_0_to_ref_rotation_w',

            'in_collision'
        ]
        return state_dict_keys


    def _check_state_dict_keys(self, state_dict) -> None:
        keys = self._get_robot_server_composition()
        if not len(keys) == len(state_dict.keys()):
            raise InvalidStateError('Robot Server state keys to not match. Different lengths.')

        for key in keys:
            if key not in state_dict.keys():
                raise InvalidStateError('Robot Server state keys to not match')


    def _get_random_ee_trans(self):
        counter = 1
        while True:
            print('_get_random_ee_trans::trying {}th trans'.format(counter))
            rand_trans = [
                uniform(-0.4, 0.4),
                uniform(-0.4, 0.4),
                uniform(0, 0.4)
            ]
            if self._check_valid_ee_trans(rand_trans):
                return rand_trans
            counter += 1


    '''
    0.1 < distance from base < 0.4
    '''
    def _check_valid_ee_trans(self, trans):
        distance = np.linalg.norm(trans)
        return 0.1 < distance < 0.4
