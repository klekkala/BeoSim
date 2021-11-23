#!/usr/bin/env python

import rospy
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
from gazebo_msgs.msg import ModelStates, ModelState


robot_model_name = 'robot'


'''
Replaces odometry in the differential drive mobile base and publishes world->base_link TF
    also updates world->objects TF
'''
class TFManager:
    def __init__(self):
        self.object_model_names = self.object_model_names = rospy.get_param('object_model_names')

        rospy.Subscriber('/gazebo/model_states', ModelStates, self.tf_handler)
        update_rate = float(rospy.get_param('~update_rate'))
        self.check_timeout = rospy.Duration(1.0 / update_rate)
        self.prev_check_time = rospy.Time.now()
        self.tf2_broadcaster = TransformBroadcaster()

        # publish once at start so tf is not empty
        model_states = rospy.wait_for_message('/gazebo/model_states', ModelStates)
        self._update_robot_tf(model_states)
        self._update_objects_tf(model_states)


    def tf_handler(self, model_states):
        self._update_robot_tf(model_states)

        time_now = rospy.Time.now()
        if time_now - self.prev_check_time < self.check_timeout:
            return
        else:
            self.prev_check_time = time_now
            self._update_objects_tf(model_states)


    def _update_robot_tf(self, model_states):
        robot_index = model_states.name.index(robot_model_name)
        robot_pose = model_states.pose[robot_index]
        t = TransformStamped()
        t.header.stamp = rospy.Time.now()
        t.header.frame_id = 'world'
        t.child_frame_id = 'dummy_link'
        t.transform.translation.x = robot_pose.position.x
        t.transform.translation.y = robot_pose.position.y
        t.transform.translation.z = robot_pose.position.z
        t.transform.rotation = robot_pose.orientation
        self.tf2_broadcaster.sendTransform(t)


    def _update_objects_tf(self, model_states):
        for object_name in self.object_model_names:
            try:
                object_index = model_states.name.index(object_name)
                object_pose = model_states.pose[object_index]
                t = TransformStamped()
                t.header.stamp = rospy.Time.now()
                t.header.frame_id = 'world'
                t.child_frame_id = object_name
                t.transform.translation.x = object_pose.position.x
                t.transform.translation.y = object_pose.position.y
                t.transform.translation.z = object_pose.position.z
                t.transform.rotation = object_pose.orientation
                self.tf2_broadcaster.sendTransform(t)
            except ValueError:
                rospy.logerr('Object %s not found in /gazebo/model_states', object_name)


if __name__ == '__main__':
    try:
        rospy.init_node('tf_manager')
        tfm = TFManager()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
