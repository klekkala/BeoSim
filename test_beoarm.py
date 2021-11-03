import gym
from robo_gym.wrappers.exception_handling import ExceptionHandling

target_machine_add = '127.0.0.1'

env = gym.make('BeoarmEEPosition-v0', gui=True, ip=target_machine_add)
env = ExceptionHandling(env)

# env.reset(target_object_position=[0.35,0,0.05])
env.reset(target_object_position=[0.2]*3)

def basic_test():
    while True:
        state, reward, _, info = env.step([0,0,0.5,0.5])
        print('joint states: {}'.format(state))

        state_dict = info['state_dict']

        object_0_trans = [
            state_dict['object_0_to_ref_translation_x'],
            state_dict['object_0_to_ref_translation_y'],
            state_dict['object_0_to_ref_translation_z']
        ]
        print('target_object_position: {}'.format(object_0_trans))

        ee_trans = [
            state_dict['ee_to_ref_translation_x'],
            state_dict['ee_to_ref_translation_y'],
            state_dict['ee_to_ref_translation_z']
        ]
        print('end_effector_position: {}'.format(ee_trans))
        print('reward(dictated by ee-object distance): {}'.format(reward))

def moveit_test():
    target_position = [
        env.state_dict['object_0_to_ref_translation_x'],
        env.state_dict['object_0_to_ref_translation_y'],
        env.state_dict['object_0_to_ref_translation_z']
    ]
    print('planning to target position {}'.format(target_position))

    moveit_plan = env.plan()
    print('plan consisted of %d points:'%len(moveit_plan))
    for p in moveit_plan:
        print('    {}'.format(p))

    env.execute_plan(moveit_plan, skip_to_end=True)

moveit_test()
