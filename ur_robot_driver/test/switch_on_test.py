#!/usr/bin/env python
import sys
import time
import unittest


import rospy
import rostopic


PKG = 'ur_robot_driver'
NAME = 'switch_on_test'

import actionlib
from ur_dashboard_msgs.msg import SetModeAction, SetModeGoal, RobotMode


class IOTest(unittest.TestCase):
    def __init__(self, *args):
        super(IOTest, self).__init__(*args)

        rospy.init_node('switch_on_robot')
        self.client = actionlib.SimpleActionClient(
            '/ur_hardware_interface/set_mode', SetModeAction)
        timeout = rospy.Duration(30)
        try:
            self.client.wait_for_server(timeout)
        except rospy.exceptions.ROSException as err:
            self.fail(
                "Could not reach set_mode action. Make sure that the driver is actually running."
                " Msg: {}".format(err))

    def test_switch_on(self):
        """Test to set an IO and check whether it has been set."""

        goal = SetModeGoal()
        goal.target_robot_mode = RobotMode.RUNNING
        goal.play_program = False # we use headless mode during tests

        self.client.send_goal(goal)
        self.client.wait_for_result()

        self.assertTrue(self.client.get_result().success)


if __name__ == '__main__':
    import rostest
    rostest.run(PKG, NAME, IOTest, sys.argv)
