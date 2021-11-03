#!/usr/bin/env python

import rospy
from std_msgs.msg import Header
from trajectory_msgs.msg import JointTrajectoryPoint, JointTrajectory

rospy.init_node('cmd_publisher')

def move_around():
    pub = rospy.Publisher('/arm_controller/command', JointTrajectory, queue_size=10)
    msg = JointTrajectory()
    msg.header = Header()
    msg.joint_names = ['joint_1', 'joint_2', 'joint_3', 'joint_4']
    # msg.joint_names = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
    msg.points = [JointTrajectoryPoint()]
    msg.points[0].positions = [3.14, 0, 3.14, 3.14]
    # msg.points[0].positions = [0, -1, 0, 0, 0, 0]
    msg.points[0].time_from_start = rospy.Duration.from_sec(10)
    r = rospy.Rate(100)
    while not rospy.is_shutdown():
        pub.publish(msg)
        r.sleep()

if __name__ == '__main__':
    move_around()
