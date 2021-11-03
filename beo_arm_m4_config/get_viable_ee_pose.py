#!/usr/bin/env python

import rospy
import rospkg
import sys
import moveit_commander
import pickle

rospack = rospkg.RosPack()
beoarm_config_path = rospack.get_path('beo_arm_m4_config')

moveit_commander.roscpp_initialize(sys.argv)
arm = moveit_commander.MoveGroupCommander('arm')

'''
usage: mvit_rand_valid_ee_pose(500) tries 500 ee poses;
        if a plan can be successfully generated for a pose, the pose is considered valid;
        all valid poses are stored in a list of PostStamped objects and pickled to <your path to beo_arm_m4_config>/ee_poses.p;

In my experience 1 in 10 poses is valid, so if you want 50 valid poses, try 500 iterations
'''
def mvit_rand_valid_ee_pose(iteration=500):
    ee_poses = []
    success_count = 0
    failure_count = 0
    try:
        for i in range(iteration):
            rand_pose = arm.get_random_pose(arm.get_end_effector_link())
            arm.set_pose_target(rand_pose)
            plan_success, plan, _, _ = arm.plan()
            if plan_success:
                ee_poses.append(rand_pose)
                success_count += 1
            else:
                failure_count += 1
            print('Iteration {} {}'.format(i, 'succeeded' if plan_success else 'failed'))
        print('Of all {} iterations {} succeeded {} failed'.format(success_count+failure_count, success_count, failure_count))
        print('Viable poses are: {}'.format(ee_poses))
        pickle.dump(ee_poses, open(beoarm_config_path+'/rand_valid_poses.p', 'wb'))
    except KeyboardInterrupt:
        rospy.signal_shutdown('KeyboardInterrupt')
        raise

'''
INPUT: takes in a list of PostStamped objects, so you may pickle.load rand_valid_poses.p and use that as the argument
OUTPUT: returns a list of tuples (PostStamped, JointTrajectory), i.e. poses and their corresponding plans;
        some poses, though valid as proven in mvit_rand_valid_ee_pose, cannot be found a plan; these poses are discarded
'''
def get_plans(poses):
    result = []
    for i, p in enumerate(poses):
        arm.set_pose_target(p)
        for j in range(1, 11):
            plan_success, plan1, _, _ = arm.plan()
            if plan_success:
                print('pose %d succeeded!'%i)
                result.append((p, plan1))
                break
            print('pose %d failed, retrying %d/10...'%(i, j))
    return result

'''
Example use:
    run "roslaunch beo_arm_m4_config demo_gazebo.launch gazebo_gui:=false" in another terminal before you run this

    valid poses that are used to reset the box target are in ee_poses.p
    list of poses-plans that is queried by ros_bridge on plan request are in ee_poses_plans.p

WARNING: This script OVERWRITES, so be sure to rename or backup before you run this
'''
if __name__ == '__main__':
    mvit_rand_valid_ee_pose(100)
    poses = pickle.load(open(beoarm_config_path+'/rand_valid_poses.p', 'rb'))
    poses_plans = get_plans(poses)
    pickle.dump(poses_plans, open(beoarm_config_path+'/ee_poses_plans.p', 'wb'))
    '''
    NOTE: eventually we don't use query on PoseStamped objects, but on Point (i.e. trans_x, trans_y, trans_z) objects
    '''
    valid_trans = [poses_plans[i][0] for i in range(len(poses_plans))]
    pickle.dump(valid_trans, open(beoarm_config_path+'/ee_poses.p', 'wb'))
