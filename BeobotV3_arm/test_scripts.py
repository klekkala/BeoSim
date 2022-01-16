#!/usr/bin/env python

import rospy
import rospkg
from std_msgs.msg import Header
from trajectory_msgs.msg import JointTrajectoryPoint, JointTrajectory
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SpawnModel, SetModelState
from geometry_msgs.msg import Pose, Point, Quaternion, Twist, Vector3

rospy.init_node('cmd_publisher')
rospack = rospkg.RosPack()

rospy.wait_for_service('/gazebo/set_model_state')
set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)

def move_arm():
    pub = rospy.Publisher('/arm_controller/command', JointTrajectory, queue_size=10)
    msg = JointTrajectory()
    msg.header = Header()
    msg.joint_names = ['arm_joint_1', 'arm_joint_2', 'arm_joint_3', 'hand_joint']
    # msg.joint_names = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
    msg.points = [JointTrajectoryPoint()]
    msg.points[0].positions = [3.14, 0, 3.14, 3.14]
    # msg.points[0].positions = [0, -1, 0, 0, 0, 0]
    msg.points[0].time_from_start = rospy.Duration.from_sec(10)
    r = rospy.Rate(100)
    while not rospy.is_shutdown():
        pub.publish(msg)
        r.sleep()

def move_base():
    pub = rospy.Publisher('/mobile_base_controller/cmd_vel', Twist, queue_size=10)
    msg = Twist()
    msg.linear = Vector3(x=0.5, y=0, z=0)
    msg.angular = Vector3(x=0, y=0, z=5)
    r = rospy.Rate(100)
    while not rospy.is_shutdown():
        pub.publish(msg)
        r.sleep()

def move_base_arm():
    arm_pub = rospy.Publisher('/arm_controller/command', JointTrajectory, queue_size=10)
    base_pub = rospy.Publisher('/mobile_base_controller/cmd_vel', Twist, queue_size=10)
    arm_msg = JointTrajectory()
    arm_msg.header = Header()
    arm_msg.joint_names = ['arm_joint_1', 'arm_joint_2', 'arm_joint_3', 'hand_joint']
    arm_msg.points = [JointTrajectoryPoint()]
    arm_msg.points[0].positions = [3.14, 0, 3.14, 3.14]
    arm_msg.points[0].time_from_start = rospy.Duration.from_sec(10)
    base_msg = Twist()
    base_msg.linear = Vector3(x=0.5, y=0, z=0)
    base_msg.angular = Vector3(x=0, y=0, z=5)
    r = rospy.Rate(100)
    while not rospy.is_shutdown():
        arm_pub.publish(arm_msg)
        base_pub.publish(base_msg)
        r.sleep()

def spawn_box(position):
    model_path = rospack.get_path('BeobotV3_robot_server') + '/models/box100/box100.sdf'
    spawn_proxy = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel, persistent=False)
    spawn_result = spawn_proxy(model_name='box100',
                                model_xml=open(model_path,'r').read(),
                                initial_pose=Pose(
                                    position=Point(
                                        position[0],
                                        position[1],
                                        position[2]
                                    ),
                                    orientation=Quaternion(0,0,0,1)
                                ),
                                reference_frame='world')

def reset_box(position):
    state_msg = ModelState()
    state_msg.model_name = 'box100'

    state_msg.pose.position.x = position[0]
    state_msg.pose.position.y = position[1]
    state_msg.pose.position.z = position[2]
    state_msg.pose.orientation.w = 0
    state_msg.pose.orientation.x = 0
    state_msg.pose.orientation.y = 0
    state_msg.pose.orientation.z = 1

    return set_state(state_msg)

if __name__ == '__main__':
    spawn_box([0.3, 0, 0.5])
    move_base()
