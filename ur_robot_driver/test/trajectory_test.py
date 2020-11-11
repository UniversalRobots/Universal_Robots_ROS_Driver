#!/usr/bin/env python
import sys
import time
import unittest

import rospy
import actionlib
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal, FollowJointTrajectoryResult
from ur_dashboard_msgs.msg import SetModeAction, SetModeGoal, RobotMode
from std_srvs.srv import Trigger, TriggerRequest
from trajectory_msgs.msg import JointTrajectoryPoint

PKG = 'ur_robot_driver'
NAME = 'trajectory_test'


class TrajectoryTest(unittest.TestCase):
    def __init__(self, *args):
        super(TrajectoryTest, self).__init__(*args)
        rospy.init_node('trajectory_testing_client')
        self.client = actionlib.SimpleActionClient(
            'follow_joint_trajectory', FollowJointTrajectoryAction)
        timeout = rospy.Duration(30)
        try:
            self.client.wait_for_server(timeout)
        except rospy.exceptions.ROSException as err:
            self.fail(
                "Could not reach controller action. Make sure that the driver is actually running."
                " Msg: {}".format(err))

        self.init_robot()

    def init_robot(self):
        """Make sure the robot is booted and ready to receive commands"""
        mode_client = actionlib.SimpleActionClient(
            '/ur_hardware_interface/set_mode', SetModeAction)
        timeout = rospy.Duration(30)
        try:
            mode_client.wait_for_server(timeout)
        except rospy.exceptions.ROSException as err:
            self.fail(
                "Could not reach set_mode action. Make sure that the driver is actually running."
                " Msg: {}".format(err))
        goal = SetModeGoal()
        goal.target_robot_mode = RobotMode.RUNNING
        goal.play_program = False # we use headless mode during tests

        mode_client.send_goal(goal)
        mode_client.wait_for_result()

        self.assertTrue(mode_client.get_result().success)

        send_program_srv = rospy.ServiceProxy("/ur_hardware_interface/resend_robot_program", Trigger)
        send_program_srv.call()
        rospy.sleep(5)

    def test_trajectory(self):
        """Test robot movement"""
        goal = FollowJointTrajectoryGoal()

        goal.trajectory.joint_names = ["elbow_joint", "shoulder_lift_joint", "shoulder_pan_joint",
                                       "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]
        position_list = [[0.0 for i in range(6)]]
        position_list.append([-0.5 for i in range(6)])
        position_list.append([-1.0 for i in range(6)])
        duration_list = [6.0, 9.0, 12.0]

        for i, position in enumerate(position_list):
            point = JointTrajectoryPoint()
            point.positions = position
            point.time_from_start = rospy.Duration(duration_list[i])
            goal.trajectory.points.append(point)

        rospy.loginfo("Sending simple goal")

        self.client.send_goal(goal)
        self.client.wait_for_result()

        self.assertEqual(self.client.get_result().error_code,
                         FollowJointTrajectoryResult.SUCCESSFUL)
        rospy.loginfo("Received result SUCCESSFUL")

    def test_illegal_trajectory(self):
        """Test trajectory server. This is more of a validation test that the testing suite does the
        right thing."""
        goal = FollowJointTrajectoryGoal()

        goal.trajectory.joint_names = ["elbow_joint", "shoulder_lift_joint", "shoulder_pan_joint",
                                       "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]
        position_list = [[0.0 for i in range(6)]]
        position_list.append([-0.5 for i in range(6)])
        # Create illegal goal by making the second point come earlier than the first
        duration_list = [6.0, 3.0]

        for i, position in enumerate(position_list):
            point = JointTrajectoryPoint()
            point.positions = position
            point.time_from_start = rospy.Duration(duration_list[i])
            goal.trajectory.points.append(point)

        rospy.loginfo("Sending illegal goal")
        self.client.send_goal(goal)
        self.client.wait_for_result()

        # As timings are illegal, we expect the result to be INVALID_GOAL
        self.assertEqual(self.client.get_result().error_code,
                         FollowJointTrajectoryResult.INVALID_GOAL)
        rospy.loginfo("Received result INVALID_GOAL")

    def test_scaled_trajectory(self):
        """Test robot movement"""
        goal = FollowJointTrajectoryGoal()

        goal.trajectory.joint_names = ["elbow_joint", "shoulder_lift_joint", "shoulder_pan_joint",
                                       "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]
        position_list = [[0.0 for i in range(6)]]
        position_list.append([-1.0 for i in range(6)])
        duration_list = [6.0, 6.5]

        for i, position in enumerate(position_list):
            point = JointTrajectoryPoint()
            point.positions = position
            point.time_from_start = rospy.Duration(duration_list[i])
            goal.trajectory.points.append(point)

        rospy.loginfo("Sending scaled goal without time restrictions")
        self.client.send_goal(goal)
        self.client.wait_for_result()

        self.assertEqual(self.client.get_result().error_code,
                         FollowJointTrajectoryResult.SUCCESSFUL)
        rospy.loginfo("Received result SUCCESSFUL")

        # Now do the same again, but with a goal time constraint
        rospy.loginfo("Sending scaled goal with time restrictions")
        goal.goal_time_tolerance = rospy.Duration(0.01)
        self.client.send_goal(goal)
        self.client.wait_for_result()

        self.assertEqual(self.client.get_result().error_code,
                         FollowJointTrajectoryResult.GOAL_TOLERANCE_VIOLATED)
        rospy.loginfo("Received result GOAL_TOLERANCE_VIOLATED")


if __name__ == '__main__':
    import rostest
    rostest.run(PKG, NAME, TrajectoryTest, sys.argv)
