import gym
import numpy as np
from grpc import RpcError
from robo_gym.utils.exceptions import InvalidStateError, RobotServerError

class MoveEffectorToWayPoints(gym.Wrapper):
    """
    Add environment a goal that the robot end-effector must reach all waypoints.
    """
    def __init__(self, env, wayPoints: np.ndarray, endEffectorName: str, distanceThreshold: float=0.3):
        """
        env: the environment to be wrapped.
        wayPoints: a (, 3) numpy array representing the (x,y,z) positions of the wayPoints.
        endEffectorName: the name of the end-effector. It must exist in the environment observation.
        distanceThreshold: the euclidean distance between the object and the target must be less than this
                         value to be considered as goal. 0.3 by default.
        """
        if not isinstance(wayPoints, np.ndarray):
            raise Exception('wayPoints must be a numpy array with shape (, 3).')
        if not isinstance(endEffectorName, str):
            raise Exception('endEffectorName must be a string.')
        if not isinstance(distanceThreshold, float) and distanceThreshold <= 0:
            raise Exception('distanceThreshold must be a positive float.')

        super().__init__(env)
        self.env = env
        self.wayPoints = np.copy(wayPoints)
        self.reachedWayPoints = [False] * len(self.wayPoints)
        self.reachCount = 0
        self.endEffectorName = endEffectorName
        self.distanceThreshold = distanceThreshold
        self.goalReached = False

        objectDict = self.client.get_state_msg().state_dict
        if self.endEffectorName + '_x' not in objectDict:
            raise Exception('{}_x does not exist in the environment.'.format(endEffectorName))
        if self.endEffectorName + '_y' not in objectDict:
            raise Exception('{}_y does not exist in the environment.'.format(endEffectorName))
        if self.endEffectorName + '_z' not in objectDict:
            raise Exception('{}_z does not exist in the environment.'.format(endEffectorName))
    
    def step(self, action):
        """
        Update the envirnoment with robot's action. If the effector's position is near any of the 
        waypoints, that waypoint is considered reached, info['atWayPoint'] will contains the 
        position of the reached waypoint(s). If all waypoints are reached, it
        is considered the goal state, and return reward = 1.
        """
        observation, reward, done, info = self.env.step(action)
        effectorPosition = np.array([observation[self.endEffectorName + '_x'],
                                     observation[self.endEffectorName + '_y'],
                                     observation[self.endEffectorName + '_z']])
        reachWayPoints = []
        for i in range(len(self.wayPoints)):
            distance = np.linalg.norm(effectorPosition - self.wayPoints[i])
            if distance <= self.distanceThreshold and not self.reachedWayPoints[i]:
                self.reachedWayPoints[i] = True
                self.reachCount += 1
                reachWayPoints.append(self.wayPoints[i])
                self.goalReached = (self.reachCount == len(self.wayPoints))

        reward = self.reward(observation, self.goalReached, info)
        info['atWaypoint'] = reachWayPoints
        return observation, reward, self.goalReached, info

    def reward(self, observation, done, info):
        """
        By default, reward = +1 when all waypoints are reached, -0.01 else.
        You can set your own reward by first derive from this class, then redefine this function.
        """
        if self.goalReached:
            return 1
        return -0.01

    def reset(self, **kwargs):
    	self.goalReached = False
    	self.reachCount = 0
    	self.reachedWayPoints = [False] * len(self.wayPoints)
    	return self.env.reset(**kwargs)

class MoveObjectToTargetTask(gym.Wrapper):
    """
    Add environment a goal that an object must be moved to the target position by all means.

    """
    def __init__(self, env, objectName: str, targetPosition: np.ndarray, distanceThreshold: float=0.3):
        """
        env: the environment to be wrapped.
        objectName: the name of the object. This object must exist in the environment, or
                    it will raise an error.
        targetPosition: an np.array with 3 elements representing (x,y,z) of the target position.
        distanceThreshold: the euclidean distance between the object and the target must be less than this
                         value to be considered as goal. 0.3 by default.

        """
        if not isinstance(objectName, str):
            raise Exception('objectName must be a string.')
        if not isinstance(targetPosition, np.ndarray) and len(targetPosition) != 3:
            raise Exception('targetPosition must be a np.array with 3 elements \
                            representing (x,y,z) of destination.')
        if not isinstance(distanceThreshold, float) and distanceThreshold <= 0:
            raise Exception('targetTolerance must be a positive float.')

        super().__init__(env)
        self.env = env
        self.objectName = objectName
        self.targetPosition = np.copy(targetPosition)
        self.distanceThreshold = distanceThreshold
        self.goalReached = False

        objectDict = self.client.get_state_msg().state_dict
        if self.objectName + '_x' not in objectDict:
           raise Exception("ObjectName {} does not exist in the environment.".format(self.objectName))

    def step(self, action):
        """
        Perform distance check for each update. If the object and target is close enough, set self.goalReached to True.
        """
        observation, reward, done, info = self.env.step(action)
        objectPosition = np.array([observation[self.objectName + '_x'],
                                   observation[self.objectName + '_y'],
                                   observation[self.objectName + '_z']])
        if not self.goalReached:
            if np.linalg.norm(objectPosition - self.targetPosition) <= self.distanceThreshold:
                self.goalReached = True
        reward = self.reward(observation, self.goalReached, info)
        return observation, reward, self.goalReached, info
        
    def reward(self, observation, done, info):
        """
        By default, reward = +1 if target is at the targetPosition, -0.01 else.
        You can set your own reward by first derive from this class, then redefine this function.
        """
        if self.goalReached:
            return 1
        return -0.01

    def reset(self, **kwargs):
    	self.goalReached = False
    	return self.env.reset(**kwargs)


    
