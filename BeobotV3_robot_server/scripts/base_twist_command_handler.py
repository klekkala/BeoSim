#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from queue import Queue

base_ctrlr_name = 'mobile_base_controller'

class BaseTwistCH:
    def __init__(self):
        rospy.init_node('base_twist_command_handler')
        ac_rate = rospy.get_param("action_cycle_rate")
        self.rate = rospy.Rate(ac_rate)
        self.bt_pub = rospy.Publisher(base_ctrlr_name+'/cmd_vel', Twist, queue_size=10)
        rospy.Subscriber('env_base_command', Twist, self.callback_env_base_twist, queue_size=1)
        self.queue = Queue(maxsize=1)
        self.stop_flag = False
        rospy.loginfo('base_twist_command_handler started')

    def callback_env_base_twist(self, data):
        try:
            self.queue.put(data)
        except:
            pass

    def base_twist_publisher(self):
        while not rospy.is_shutdown():
            if self.queue.full():
                self.bt_pub.publish(self.queue.get())
                self.stop_flag = False
            else:
                if not self.stop_flag:
                    self.bt_pub.publish(Twist())
                    self.stop_flag = True
                else:
                    pass
            self.rate.sleep()


if __name__ == '__main__':
    try:
        ch = BaseTwistCH()
        ch.base_twist_publisher()
    except rospy.ROSInterruptException:
        pass
