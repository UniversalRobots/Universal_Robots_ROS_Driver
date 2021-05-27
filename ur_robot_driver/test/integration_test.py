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
from ur_msgs.srv import SetIO, SetIORequest
from ur_msgs.msg import IOStates

PKG = 'ur_robot_driver'
NAME = 'integration_test'


class IntegrationTest(unittest.TestCase):
    def __init__(self, *args):
        super(IntegrationTest, self).__init__(*args)

        self.init_robot()

    def init_robot(self):
        """Make sure the robot is booted and ready to receive commands"""

        rospy.init_node('ur_robot_driver_integration_test')
        timeout = rospy.Duration(30)

        self.set_mode_client = actionlib.SimpleActionClient(
            '/ur_hardware_interface/set_mode', SetModeAction)
        try:
            self.set_mode_client.wait_for_server(timeout)
        except rospy.exceptions.ROSException as err:
            self.fail(
                "Could not reach set_mode action. Make sure that the driver is actually running."
                " Msg: {}".format(err))

        self.trajectory_client = actionlib.SimpleActionClient(
            'follow_joint_trajectory', FollowJointTrajectoryAction)
        try:
            self.trajectory_client.wait_for_server(timeout)
        except rospy.exceptions.ROSException as err:
            self.fail(
                "Could not reach controller action. Make sure that the driver is actually running."
                " Msg: {}".format(err))

        self.set_io_client = rospy.ServiceProxy('/ur_hardware_interface/set_io', SetIO)
        try:
            self.set_io_client.wait_for_service(timeout)
        except rospy.exceptions.ROSException as err:
            self.fail(
                "Could not reach SetIO service. Make sure that the driver is actually running."
                " Msg: {}".format(err))

        self.send_program_srv = rospy.ServiceProxy('/ur_hardware_interface/resend_robot_program',
                                                   Trigger)
        try:
            self.send_program_srv.wait_for_service(timeout)
        except rospy.exceptions.ROSException as err:
            self.fail(
                "Could not reach resend_robot_program service. Make sure that the driver is "
                "actually running in headless mode."
                " Msg: {}".format(err))

    def set_robot_to_mode(self, target_mode):
        goal = SetModeGoal()
        goal.target_robot_mode = target_mode
        goal.play_program = False # we use headless mode during tests
        # This might be a bug to hunt down. We have to reset the program before calling `resend_robot_program`
        goal.stop_program = True

        self.set_mode_client.send_goal(goal)
        self.set_mode_client.wait_for_result()
        return self.set_mode_client.get_result().success


    def test_trajectories(self):
        """Test robot movement"""
        #### Power cycle the robot in order to make sure it is running correctly####
        self.assertTrue(self.set_robot_to_mode(RobotMode.POWER_OFF))
        rospy.sleep(0.5)
        self.assertTrue(self.set_robot_to_mode(RobotMode.RUNNING))
        rospy.sleep(0.5)

        self.send_program_srv.call()
        rospy.sleep(0.5) # TODO properly wait until the controller is running

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

        self.trajectory_client.send_goal(goal)
        self.trajectory_client.wait_for_result()

        self.assertEqual(self.trajectory_client.get_result().error_code,
                         FollowJointTrajectoryResult.SUCCESSFUL)
        rospy.loginfo("Received result SUCCESSFUL")

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
        self.trajectory_client.send_goal(goal)
        self.trajectory_client.wait_for_result()

        # As timings are illegal, we expect the result to be INVALID_GOAL
        self.assertEqual(self.trajectory_client.get_result().error_code,
                         FollowJointTrajectoryResult.INVALID_GOAL)
        rospy.loginfo("Received result INVALID_GOAL")

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
        self.trajectory_client.send_goal(goal)
        self.trajectory_client.wait_for_result()

        self.assertEqual(self.trajectory_client.get_result().error_code,
                         FollowJointTrajectoryResult.SUCCESSFUL)
        rospy.loginfo("Received result SUCCESSFUL")

        # Now do the same again, but with a goal time constraint
        rospy.loginfo("Sending scaled goal with time restrictions")
        goal.goal_time_tolerance = rospy.Duration(0.01)
        self.trajectory_client.send_goal(goal)
        self.trajectory_client.wait_for_result()

        self.assertEqual(self.trajectory_client.get_result().error_code,
                         FollowJointTrajectoryResult.GOAL_TOLERANCE_VIOLATED)
        rospy.loginfo("Received result GOAL_TOLERANCE_VIOLATED")


    def test_set_io(self):
        """Test to set an IO and check whether it has been set."""
        maximum_messages = 5
        pin = 0
        self.assertEqual(maximum_messages, 5)

        self.set_io_client(1, pin, 0)
        messages = 0
        pin_state = True

        while(pin_state):
            if messages >= maximum_messages:
                self.fail("Could not read desired state after {} messages.".format(maximum_messages))
            io_state = rospy.wait_for_message('/ur_hardware_interface/io_states', IOStates)
            pin_state = io_state.digital_out_states[pin].state
            messages += 1
        self.assertEqual(pin_state, 0)

        self.set_io_client(SetIORequest.FUN_SET_DIGITAL_OUT, pin, 1)
        messages = 0
        pin_state = False

        while(not pin_state):
            if messages >= maximum_messages:
                self.fail("Could not read desired state after {} messages.".format(maximum_messages))
            io_state = rospy.wait_for_message('/ur_hardware_interface/io_states', IOStates)
            pin_state = io_state.digital_out_states[pin].state
            messages += 1
        self.assertEqual(pin_state, 1)


if __name__ == '__main__':
    import rostest
    rostest.run(PKG, NAME, IntegrationTest, sys.argv)
