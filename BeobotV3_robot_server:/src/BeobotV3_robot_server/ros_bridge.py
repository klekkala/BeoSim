#!/usr/bin/env python

import copy
import rospy
import rospkg
import tf2_ros
import numpy as np
from retry import retry
from threading import Event
from std_msgs.msg import Int32MultiArray, Header, Bool
from geometry_msgs.msg import Pose, Point, Quaternion, Twist, Vector3, TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from controller_manager_msgs.srv import SwitchController, LoadController
from gazebo_msgs.msg import ContactsState, ModelState, ModelStates
from gazebo_msgs.srv import DeleteModel, SpawnModel, SetModelState
from trajectory_msgs.msg import JointTrajectoryPoint, JointTrajectory
from robo_gym_server_modules.robot_server.grpc_msgs.python import robot_server_pb2
from tf.transformations import euler_from_quaternion

'''
state_dict should look like this:
[
    'arm_joint_1_position',
    'arm_joint_2_position',
    'arm_joint_3_position',
    'hand_joint_position',

    'arm_joint_1_velocity',
    'arm_joint_2_velocity',
    'arm_joint_3_velocity',
    'hand_joint_velocity',

    'base_pose_x',
    'base_pose_y',
    'base_pose_yaw',

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
'''

robot_model_name = 'robot'
rospack = rospkg.RosPack()
robot_urdf_path = rospack.get_path('BeobotV3_arm') + '/urdf/BeobotV3_arm.urdf'
robot_spawn_position = [0, 0, 0.084]
object_sdf_path = rospack.get_path('BeobotV3_robot_server') + '/models/box100/box100.sdf'
manipulator_group_name = 'arm'
manipulator_ctrlr_name = 'arm_controller'
base_ctrlr_name = 'mobile_base_controller'
tf_lookup_attempts = 10

class RosBridge:

    def __init__(self):
        # Event is clear while initialization or set_state is going on
        self.reset = Event()
        self.reset.clear()
        self.get_state_event = Event()
        self.get_state_event.set()

        # Arm joint states
        self.joint_names = ['arm_joint_1', 'arm_joint_2', 'arm_joint_3', 'hand_joint']
        self.joint_position = dict.fromkeys(self.joint_names, 0.0)
        self.joint_velocity = dict.fromkeys(self.joint_names, 0.0)
        rospy.Subscriber("joint_states", JointState, self._on_joint_states)

        # Robot control
        self.arm_cmd_pub = rospy.Publisher('env_arm_command', JointTrajectory, queue_size=1) # joint_trajectory_command_handler publisher
        self.base_cmd_pub = rospy.Publisher('env_base_command', Twist, queue_size=1)
        # FIXME: rospy.Rate
        self.action_cycle_rate = rospy.Rate(float(rospy.get_param("action_cycle_rate")))
        self.min_traj_duration = 0.5 # minimum trajectory duration (s)
        # FIXME: change this to reflect real velocity limits
        self.joint_velocity_limits = dict(zip(self.joint_names, [1.0] * 4))

        # Robot frames
        self.reference_frame = 'world'
        self.base_frame = 'base_link'
        self.ee_frame = 'hand_tip'
        self.object_frames = ['box100']

        self.object_0_pose = {
            'object_0_to_ref_translation_x': 0.0,
            'object_0_to_ref_translation_y': 0.0,
            'object_0_to_ref_translation_z': 0.0,
            'object_0_to_ref_rotation_x': 0.0,
            'object_0_to_ref_rotation_y': 0.0,
            'object_0_to_ref_rotation_z': 0.0,
            'object_0_to_ref_rotation_w': 0.0
        }
        rospy.Subscriber('/gazebo/model_states', ModelStates, self._update_object_pose)

        # Service proxy for setting target object states
        rospy.wait_for_service('/gazebo/set_model_state')
        self.set_state_proxy = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)

        # TF2
        self.tf2_buffer = tf2_ros.Buffer()
        self.tf2_listener = tf2_ros.TransformListener(self.tf2_buffer)
        self.tf2_broadcaster = tf2_ros.TransformBroadcaster()

        # Collision detection
        # FIXME: restart every _reset_robot_object?
        # FIXME: mobile_base?
        self.collision_subs = [
            rospy.Subscriber("hand_collision", ContactsState, self._on_hand_collision),
            rospy.Subscriber("arm_link1_collision", ContactsState, self._on_link1_collision),
            rospy.Subscriber("arm_link2_collision", ContactsState, self._on_link2_collision),
            rospy.Subscriber("arm_link3_collision", ContactsState, self._on_link3_collision),
            rospy.Subscriber("base_link_collision", ContactsState, self._on_base_collision),
        ]
        # Initialization of collision sensor flags
        self.collision_sensors = dict.fromkeys(["hand", "link_1", "link_2", "link_3", "base"], False)


    def get_state(self):
        self.get_state_event.clear()
        # Get environment state
        state_dict = {}

        # Joint Positions and Joint Velocities
        joint_position = copy.deepcopy(self.joint_position)
        joint_velocity = copy.deepcopy(self.joint_velocity)
        state_dict.update(self._get_joint_states_dict(joint_position, joint_velocity))

        # ee/obj to ref transform
        tf_lookup_succeeded = False
        for i in range(tf_lookup_attempts):
            if tf_lookup_succeeded:
                break
            try:
                base_to_ref_trans = self.tf2_buffer.lookup_transform(self.reference_frame, self.base_frame, rospy.Time(0))
                state_dict['base_pose_x'] = base_to_ref_trans.transform.translation.x
                state_dict['base_pose_y'] = base_to_ref_trans.transform.translation.y
                state_dict['base_pose_yaw'] = euler_from_quaternion([
                                        base_to_ref_trans.transform.rotation.x,
                                        base_to_ref_trans.transform.rotation.y,
                                        base_to_ref_trans.transform.rotation.z,
                                        base_to_ref_trans.transform.rotation.w])[2]

                ee_to_ref_trans = self.tf2_buffer.lookup_transform(self.reference_frame, self.ee_frame, rospy.Time(0))
                state_dict.update(self._get_transform_dict(ee_to_ref_trans, 'ee_to_ref'))

                state_dict.update(self.object_0_pose)

                tf_lookup_succeeded = True
            except (tf2_ros.LookupException, tf2_ros.ExtrapolationException):
                rospy.logwarn('TF lookup failed, retrying...')
                rospy.sleep(1)
        if not tf_lookup_succeeded:
            return robot_server_pb2.State(success=False)

        # Collision sensors
        beobot_collision = any(self.collision_sensors.values())
        state_dict['in_collision'] = float(beobot_collision)

        self.get_state_event.set()

        # Create and fill State message
        msg = robot_server_pb2.State(state_dict=state_dict, success=True)

        return msg


    '''
    resets object_0 position to that given in float_params and robot base pose & joint positions to zero state

    does this by deleting and respawning
        - tried SetModelConfiguration, doesn't work, which is also why this function currently doesn't support setting to non-zero state
    '''
    def reset_robot_object(self, state_msg):
        self.reset.clear()

        # ignores joint position information in state_msg

        # FIXME: support for more objects?
        try:
            object_0_x = state_msg.float_params['object_0_x']
            object_0_y = state_msg.float_params['object_0_y']
            object_0_z = state_msg.float_params['object_0_z']
            self._set_object_position(self.object_frames[0], [object_0_x, object_0_y, object_0_z])
        except KeyError:
            rospy.logwarn('No object_0 reset position given, skipping...')
        except rospy.ServiceException:
            rospy.logerr('reset_robot_object::set_object_position failed!')
            self.reset.set()
            self.action_cycle_rate.sleep()
            return 0

        try:
            rospy.loginfo('reset_robot_object::deleting robot %s...', robot_model_name)
            self._delete_model(robot_model_name)
            rospy.loginfo('reset_robot_object::respawning robot %s from urdf %s...', robot_model_name, robot_urdf_path)
            self._respawn_model(robot_model_name, robot_urdf_path, robot_spawn_position)
            rospy.loginfo('reset_robot_object::loading base controller %s...', base_ctrlr_name)
            self._load_base_ctrl()
            rospy.loginfo('reset_robot_object::loading arm controller %s...', manipulator_ctrlr_name)
            self._load_arm_ctrl()
            rospy.loginfo('reset_robot_object::loading joint state controller...')
            self._load_jst()
            rospy.loginfo('reset_robot_object::activating controllers...')
            self._activate_controllers()
        except rospy.ServiceException:
            rospy.logerr('reset_robot_object:: deleting and respawning robot failed!')
            self.reset.set()
            self.action_cycle_rate.sleep()
            return 0

        # restart collision subscribers every iter to prevent collision states from the previous iter leaking into this iter
        #   (not sure if it's really what's happening but restarting does seem to reduce errors)
        for s in self.collision_subs: s.unregister();
        self.collision_subs = [
            rospy.Subscriber("hand_collision", ContactsState, self._on_hand_collision),
            rospy.Subscriber("arm_link1_collision", ContactsState, self._on_link1_collision),
            rospy.Subscriber("arm_link2_collision", ContactsState, self._on_link2_collision),
            rospy.Subscriber("arm_link3_collision", ContactsState, self._on_link3_collision),
            rospy.Subscriber("base_link_collision", ContactsState, self._on_base_collision),
        ]

        self.collision_sensors.update(dict.fromkeys(["hand", "link_1", "link_2", "link_3", "base"], False))

        self.reset.set()
        self.action_cycle_rate.sleep()

        return 1


    '''
    action_cmd should look like this:
        ['arm_joint_1_pos', 'arm_joint_2_pos', 'arm_joint_3_pos', 'hand_joint_pos', 'base_twist_linear_x', 'base_twist_angular_z']
    '''
    def base_and_arm_action(self, action_cmd, guarantee=False):
        assert len(action_cmd) == (len(self.joint_names) + 2)
        joint_positions = action_cmd[:len(self.joint_names)]
        base_twist = action_cmd[len(self.joint_names):]

        self._publish_env_base_cmd(base_twist)

        if guarantee:
            self._set_joint_position(joint_positions)
        else:
            self._publish_env_arm_cmd(joint_positions)
        self.action_cycle_rate.sleep()
        return action_cmd


    def _set_joint_position(self, goal_joint_position):
        """Set robot joint positions to a desired value
        """
        assert len(goal_joint_position) == len(self.joint_names)
        # FIXME: ros action?
        position_reached = False
        rospy.loginfo('Setting joint positions:')
        rospy.loginfo(goal_joint_position)
        while not position_reached:
            self._publish_env_arm_cmd(goal_joint_position)
            self.get_state_event.clear()
            joint_position = list(copy.deepcopy(self.joint_position).values())
            position_reached = np.isclose(goal_joint_position, joint_position, atol=0.03).all()
            self.get_state_event.set()

    def _publish_env_arm_cmd(self, position_cmd):
        """Publish environment JointTrajectory msg.
        """
        assert len(position_cmd) == len(self.joint_names)
        msg = JointTrajectory()
        msg.header = Header()
        msg.joint_names = self.joint_names
        msg.points=[JointTrajectoryPoint()]
        msg.points[0].positions = position_cmd
        dur = []
        for idx, name in enumerate(msg.joint_names):
            pos = self.joint_position[name]
            cmd = position_cmd[idx]
            max_vel = self.joint_velocity_limits[name]
            dur.append(max(abs(cmd-pos)/max_vel, self.min_traj_duration))
        msg.points[0].time_from_start = rospy.Duration.from_sec(max(dur))
        self.arm_cmd_pub.publish(msg)

    def _publish_env_base_cmd(self, twist_cmd):
        assert len(twist_cmd) == 2
        msg = Twist()
        msg.linear = Vector3(x=twist_cmd[0], y=0, z=0)
        msg.angular = Vector3(x=0, y=0, z=twist_cmd[1])
        self.base_cmd_pub.publish(msg)

    def _update_object_pose(self, model_states):
        for object_name in self.object_frames:
            try:
                object_index = model_states.name.index(object_name)
                object_pose = model_states.pose[object_index]
                self.object_0_pose['object_0_to_ref_translation_x'] = object_pose.position.x
                self.object_0_pose['object_0_to_ref_translation_y'] = object_pose.position.y
                self.object_0_pose['object_0_to_ref_translation_z'] = object_pose.position.z
                self.object_0_pose['object_0_to_ref_rotation_x'] = object_pose.orientation.x
                self.object_0_pose['object_0_to_ref_rotation_y'] = object_pose.orientation.y
                self.object_0_pose['object_0_to_ref_rotation_z'] = object_pose.orientation.z
                self.object_0_pose['object_0_to_ref_rotation_w'] = object_pose.orientation.w
            except ValueError:
                rospy.logerr('Object %s not found in /gazebo/model_states', object_name)

    @retry(delay=1, tries=5)
    def _set_object_position(self, model, new_position):
        rospy.loginfo('setting object position for %s...', model)
        rospy.wait_for_service('/gazebo/set_model_state')
        state_msg = ModelState()
        state_msg.model_name = model

        state_msg.pose.position.x = new_position[0]
        state_msg.pose.position.y = new_position[1]
        state_msg.pose.position.z = new_position[2]
        state_msg.pose.orientation.w = 0
        state_msg.pose.orientation.x = 0
        state_msg.pose.orientation.y = 0
        state_msg.pose.orientation.z = 1

        result = self.set_state_proxy(state_msg).success
        if not result:
            rospy.logwarn('set_object_position failed, retrying...')
            raise rospy.ServiceException

    @retry(delay=1, tries=5)
    def _delete_model(self, model):
        rospy.wait_for_service('/gazebo/delete_model')
        # ServiceProxy gets closed after the first call when persistent=False, so no need to close()
        delete_proxy = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel, persistent=False)
        result = delete_proxy(model_name=model).success
        if not result:
            rospy.logwarn('delete_model failed, retrying...')
            raise rospy.ServiceException

    @retry(delay=1, tries=5)
    def _respawn_model(self, model, path, spawn_position, use_sdf=False):
        assert len(spawn_position) == 3
        spawn_proxy = None
        if use_sdf:
            rospy.wait_for_service('/gazebo/spawn_sdf_model')
            spawn_proxy = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel, persistent=False)
        else:
            rospy.wait_for_service('/gazebo/spawn_urdf_model')
            spawn_proxy = rospy.ServiceProxy('/gazebo/spawn_urdf_model', SpawnModel, persistent=False)
        result = spawn_proxy(model_name=model,
                            model_xml=open(path,'r').read(),
                            initial_pose=Pose(
                                position=Point(
                                    spawn_position[0],
                                    spawn_position[1],
                                    spawn_position[2]
                                ),
                                orientation=Quaternion(0,0,0,1)
                            ),
                            reference_frame=self.reference_frame).success
        if not result:
            rospy.logwarn('respawn_model failed, retrying...')
            raise rospy.ServiceException

    @retry(delay=1, tries=5)
    def _load_base_ctrl(self):
        rospy.wait_for_service('/controller_manager/load_controller')
        base_ctrlr_load_proxy = rospy.ServiceProxy('/controller_manager/load_controller', LoadController, persistent=False)
        result = base_ctrlr_load_proxy(name=base_ctrlr_name).ok
        if not result:
            rospy.logwarn('load_base_ctrl failed, retrying...')
            raise rospy.ServiceException

    @retry(delay=1, tries=5)
    def _load_arm_ctrl(self):
        rospy.wait_for_service('/controller_manager/load_controller')
        arm_ctrlr_load_proxy = rospy.ServiceProxy('/controller_manager/load_controller', LoadController, persistent=False)
        result = arm_ctrlr_load_proxy(name=manipulator_ctrlr_name).ok
        if not result:
            rospy.logwarn('load_arm_ctrl failed, retrying...')
            raise rospy.ServiceException

    @retry(delay=1, tries=5)
    def _load_jst(self):
        rospy.wait_for_service('/controller_manager/load_controller')
        jts_ctrlr_load_proxy = rospy.ServiceProxy('/controller_manager/load_controller', LoadController, persistent=False)
        result = jts_ctrlr_load_proxy(name='joint_state_controller').ok
        if not result:
            rospy.logwarn('load_jst failed, retrying...')
            raise rospy.ServiceException

    @retry(delay=1, tries=5)
    def _activate_controllers(self):
        rospy.wait_for_service('/controller_manager/switch_controller')
        controller_on_proxy = rospy.ServiceProxy('/controller_manager/switch_controller', SwitchController, persistent=False)
        result = controller_on_proxy(start_controllers=[manipulator_ctrlr_name, base_ctrlr_name, 'joint_state_controller'],
                                        stop_controllers=[],
                                        strictness=1).ok
        if not result:
            rospy.logwarn('activate_controllers failed, retrying...')
            raise rospy.ServiceException

    def _on_joint_states(self, msg):
        if self.get_state_event.is_set():
            for idx, name in enumerate(msg.name):
                if name in self.joint_names:
                    self.joint_position[name] = msg.position[idx]
                    self.joint_velocity[name] = msg.velocity[idx]

    def _on_hand_collision(self, data):
        if data.states == []:
            pass
        else:
            self.collision_sensors["hand"] = True

    def _on_link1_collision(self, data):
        if data.states == []:
            pass
        else:
            self.collision_sensors["link_1"] = True

    def _on_link2_collision(self, data):
        if data.states == []:
            pass
        else:
            self.collision_sensors["link_2"] = True

    def _on_link3_collision(self, data):
        if data.states == []:
            pass
        else:
            self.collision_sensors["link_3"] = True

    def _on_base_collision(self, data):
        if data.states == []:
            pass
        else:
            self.collision_sensors["base"] = True

    def _on_occupancy_state(self, msg):
        if self.get_state_event.is_set():
            # occupancy_3d_array = np.reshape(msg.data, [dim.size for dim in msg.layout.dim])
            self.voxel_occupancy = msg.data
        else:
            pass

    def _get_joint_states_dict(self, joint_position, joint_velocity):
        d = {}
        for joint in self.joint_names:
            d[joint+'_position'] = joint_position[joint]
            d[joint+'_velocity'] = joint_velocity[joint]
        return d

    def _get_transform_dict(self, transform, transform_name):
        d ={}
        d[transform_name + '_translation_x'] = transform.transform.translation.x
        d[transform_name + '_translation_y'] = transform.transform.translation.y
        d[transform_name + '_translation_z'] = transform.transform.translation.z
        d[transform_name + '_rotation_x'] = transform.transform.rotation.x
        d[transform_name + '_rotation_y'] = transform.transform.rotation.y
        d[transform_name + '_rotation_z'] = transform.transform.rotation.z
        d[transform_name + '_rotation_w'] = transform.transform.rotation.w
        return d
