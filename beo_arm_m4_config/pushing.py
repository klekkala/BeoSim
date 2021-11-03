#!/usr/bin/env python

import rospy
import sys
import moveit_commander
import math
import numpy as np
from tf.transformations import quaternion_from_euler
from std_msgs.msg import Header, ColorRGBA
from geometry_msgs.msg import Pose, PoseStamped
from moveit_msgs.msg import Constraints, PositionConstraint
from shape_msgs.msg import SolidPrimitive
from visualization_msgs.msg import Marker


rospy.init_node("pushing", anonymous=True)
moveit_commander.roscpp_initialize(sys.argv)
arm = moveit_commander.MoveGroupCommander('arm')
scene = moveit_commander.PlanningSceneInterface()
reference_frame = 'dummy_ground'
ee_link = 'hand_tip'
object_radius = 0.12
marker_id_counter = 0
marker_pub = rospy.Publisher("/visualization_marker", Marker, queue_size=20)


def display_cylinder(pose, dimensions):
    global marker_id_counter

    assert len(dimensions) == 2

    marker = Marker()
    marker.header.stamp = rospy.Time.now()
    marker.ns = "/"
    marker.id = marker_id_counter
    marker.type = Marker.CYLINDER
    marker.action = Marker.ADD
    marker.color = ColorRGBA(0.0, 1.0, 0.0, 0.5)
    marker.header.frame_id = reference_frame

    marker.pose = pose
    marker.scale.x = dimensions[1]
    marker.scale.y = dimensions[1]
    marker.scale.z = dimensions[0]

    marker_pub.publish(marker)
    marker_id_counter += 1


def remove_all_markers():
    marker = Marker()
    marker.header.stamp = rospy.Time.now()
    marker.ns = "/"
    marker.action = Marker.DELETEALL
    marker_pub.publish(marker)


def reset_push_plan_env():
    remove_all_markers()
    arm.clear_path_constraints()
    arm.clear_pose_targets()
    # scene.remove_world_object()
    # scene.remove_attached_object()


def push(starting_pose, target_pose):
    # reset
    reset_push_plan_env()

    # add a box to be pushed (use scene.__make_existing(name=id) if already have obj to be pushed)
    #   needed because arm cannot collide with object in phase1
    rospy.sleep(1)
    box_pose = PoseStamped()
    box_pose.header.frame_id = reference_frame
    box_pose.pose.position = starting_pose.position
    scene.add_box(name='box', pose=box_pose, size=[0.1]*3)

    # some calculations
    bound_shape_x = np.mean([starting_pose.position.x, target_pose.position.x])
    bound_shape_y = np.mean([starting_pose.position.y, target_pose.position.y])
    x_diff = float(target_pose.position.x - starting_pose.position.x)
    y_diff = float(target_pose.position.y - starting_pose.position.y)
    grad = math.atan(y_diff / x_diff) if x_diff != 0 else math.pi/2
    det = math.sqrt(x_diff**2 + y_diff**2)
    bound_shape_len = det + object_radius*3
    goal_offset_x = (object_radius/bound_shape_len)*x_diff
    goal_offset_y = (object_radius/bound_shape_len)*y_diff

    # Phase1: EE is moving to target obj's original location
    # print([
    #                             starting_pose.position.x - goal_offset_x,
    #                             starting_pose.position.y - goal_offset_y,
    #                             starting_pose.position.z
    #                         ])
    # [0.2894022007770987, 0.24239119689160515, 0.05]
    print(goal_offset_x, goal_offset_y)
    arm.set_position_target([0.35, 0.35, 0.05])
    plan_success1, plan1, _, _ = arm.plan()
    if plan_success1:
        arm.execute(plan1)
    else:
        rospy.logwarn('Push phase1 plan failed...')
        reset_push_plan_env()
        return False

    # arm can(should) collide with object in phase2, attach to ignore collision
    #   can still use this even if the existing object is not a box, just set name to id
    scene.attach_box(link=ee_link, name='box')

    # Phase2: EE has reached target obj's original location, now moving to target location in constrained path
    # add constraint
    position_constraint = PositionConstraint()
    position_constraint.header.frame_id = reference_frame
    position_constraint.link_name = ee_link
    position_constraint.weight = 1.0

    bound_shape = SolidPrimitive()
    bound_shape.type = SolidPrimitive.CYLINDER
    bound_shape.dimensions = [bound_shape_len, object_radius]
    position_constraint.constraint_region.primitives.append(bound_shape)

    bound_shape_pose = Pose()
    bound_shape_pose.position.x = bound_shape_x
    bound_shape_pose.position.y = bound_shape_y
    bound_shape_pose.position.z = starting_pose.position.z
    quat = quaternion_from_euler(0, math.pi/2, grad)
    bound_shape_pose.orientation.x = quat[0]
    bound_shape_pose.orientation.y = quat[1]
    bound_shape_pose.orientation.z = quat[2]
    bound_shape_pose.orientation.w = quat[3]
    position_constraint.constraint_region.primitive_poses.append(bound_shape_pose)

    # display constraint region
    display_cylinder(bound_shape_pose, bound_shape.dimensions)

    # set constraints, goal, and push
    constraints = Constraints()
    constraints.name = 'push_within_cylinder_bounds'
    constraints.position_constraints.append(position_constraint)

    arm.set_position_target([
                                target_pose.position.x - goal_offset_x,
                                target_pose.position.y - goal_offset_y,
                                target_pose.position.z
                            ])
    arm.set_path_constraints(constraints)
    plan_success2, plan2, _, _ = arm.plan()
    if plan_success2:
        arm.execute(plan2)
    else:
        rospy.logwarn('Push phase2 plan failed...')
        reset_push_plan_env()
        return False

    return True


if __name__ == '__main__':
    starting_pose = Pose()
    starting_pose.position.x = 0.3
    starting_pose.position.y = 0.2
    starting_pose.position.z = 0.05

    target_pose = Pose()
    target_pose.position.x = 0.35
    target_pose.position.y = 0.0
    target_pose.position.z = 0.05

    push(starting_pose, target_pose)
