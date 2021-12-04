    def reset_2(self, joint_positions,ee_target_pose):
        # self.joint_positions = joint_positions
        joint_position = JOINT_POSITIONS
        self._set_joint_positions(joint_position)
        self.elapsed_steps = 0

        rs_state_0 = dict.fromkeys(self.get_robot_server_composition(), 0.0)
        rs_state_0.update(self.joint_positions)
        state_msg = self._set_initial_robot_server_state(rs_state_0, ee_target_pose)


        if not self.client.set_state_msg(state_msg):
            raise RobotServerError("set_state")

        # self.client.set_state_msg(state_msg_old)

        rs_state_0 = self.client.get_state_msg().state_dict

        state = self._robot_server_state_to_env_state(rs_state_0)

        # Check if the environment state is contained in the observation space
        if not self.observation_space.contains(state):
            raise InvalidStateError()

        # Check if current position is in the range of the initial joint positions
        for joint in self.joint_positions.keys():
            if not np.isclose(self.joint_positions[joint], rs_state_0[joint], atol=0.05):
                raise InvalidStateError('Reset joint positions are not within defined range')

        return state

    def reset_new(self, joint_positions = JOINT_POSITIONS, ee_target_pose = None, randomize_start=False) -> np.array:
        """Environment reset.

        Args:
            joint_positions (list[6] or np.array[6]): robot joint positions in radians.
            ee_target_pose (list[6] or np.array[6]): [x,y,z,r,p,y] target end effector pose.
            randomize_start (bool): if True the starting position is randomized defined by the RANDOM_JOINT_OFFSET
        """
        if joint_positions:
            assert len(joint_positions) == 6
        else:
            joint_positions = JOINT_POSITIONS

        self.elapsed_steps = 0

        # Initialize environment state
        state_len = self.observation_space.shape[0]
        state = np.zeros(state_len)
        rs_state = dict.fromkeys(self.get_robot_server_composition(), 0.0)

        # Randomize initial robot joint positions
        if randomize_start:
            joint_positions_low = np.array(joint_positions) - np.array(RANDOM_JOINT_OFFSET)
            joint_positions_high = np.array(joint_positions) + np.array(RANDOM_JOINT_OFFSET)
            joint_positions = np.random.default_rng().uniform(low=joint_positions_low, high=joint_positions_high)

        # Set initial robot joint positions
        self._set_joint_positions(joint_positions)

        # Update joint positions in rs_state
        rs_state.update(self.joint_positions)

        # Set target End Effector pose
        if ee_target_pose:
            assert len(ee_target_pose) == 6
        else:
            ee_target_pose = self._get_target_pose()

        # Set initial state of the Robot Server
        state_msg = self._set_initial_robot_server_state(rs_state, ee_target_pose)

        if not self.client.set_state_msg(state_msg):
            raise RobotServerError("set_state")

        # Get Robot Server state
        rs_state = self.client.get_state_msg().state_dict
        # Check if the length and keys of the Robot Server state received is correct
        self._check_rs_state_keys(rs_state)
        # Convert the initial state from Robot Server format to environment format
        state = self._robot_server_state_to_env_state(rs_state)

        # Check if the environment state is contained in the observation space
        if not self.observation_space.contains(state):
            raise InvalidStateError()

        # Check if current position is in the range of the initial joint positions
        for joint in self.joint_positions.keys():
            if not np.isclose(self.joint_positions[joint], rs_state[joint], atol=0.05):
                raise InvalidStateError('Reset joint positions are not within defined range')

        return state,self.joint_positions ,ee_target_pose
