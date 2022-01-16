#!/usr/bin/env python2

import grpc
import rospy
from concurrent import futures
from BeobotV3_robot_server.ros_bridge import RosBridge
from robo_gym_server_modules.robot_server.grpc_msgs.python import robot_server_pb2, robot_server_pb2_grpc

class RobotServerServicer(robot_server_pb2_grpc.RobotServerServicer):
    def __init__(self):
        self.rosbridge = RosBridge()

    def GetState(self, request, context):
        try:
            return self.rosbridge.get_state()
        except:
            rospy.logerr('Failed to get state', exc_info=True)
            return robot_server_pb2.State(success=0)

    def SetState(self, request, context):
        return robot_server_pb2.Success(success=0)

    def ResetState(self, request, context):
        try:
            reset_result = self.rosbridge.reset_robot_object(state_msg=request)
            if reset_result:
                return robot_server_pb2.Success(success=1)
            else:
                return robot_server_pb2.Success(success=0)
        except:
            rospy.logerr('Failed to reset state', exc_info=True)
            return robot_server_pb2.Success(success=0)

    def SendAction(self, request, context):
        try:
            executed_action = self.rosbridge.base_and_arm_action(request.action)
            return robot_server_pb2.Success(success=1)
        except:
            rospy.logerr('Failed to send action', exc_info=True)
            return robot_server_pb2.Success(success=0)

    def SendActionGetState(self, request, context):
        try:
            executed_action = self.rosbridge.base_and_arm_action(request.action)
            return self.rosbridge.get_state()
        except:
            rospy.logerr('Failed to send action and get state', exc_info=True)
            return robot_server_pb2.State(success = 0)

    def SendGoalGetPlan(self, request, context):
        return robot_server_pb2.MoveitPlan(success = 0)

    def StrictActionGetState(self, request, context):
        try:
            executed_action = self.rosbridge.base_and_arm_action(request.action, True)
            return self.rosbridge.get_state()
        except:
            rospy.logerr('Failed to send strict action and get state', exc_info=True)
            return robot_server_pb2.State(success = 0)

def serve():
    rospy.loginfo('Starting BeobotV3 Robot Server...')
    server_port = rospy.get_param('~server_port')
    server = grpc.server(futures.ThreadPoolExecutor(max_workers=10))
    robot_server_pb2_grpc.add_RobotServerServicer_to_server(RobotServerServicer(), server)
    server.add_insecure_port('[::]:'+repr(server_port))
    server.start()
    rospy.loginfo('BeobotV3 Sim Robot Server started at ' + repr(server_port))
    rospy.spin()

if __name__ == '__main__':
    try:
        rospy.init_node('robot_server')
        rospy.loginfo('Waiting 10s before starting initialization Robot Server')
        rospy.sleep(10)
        serve()
    except (KeyboardInterrupt, SystemExit):
        pass
