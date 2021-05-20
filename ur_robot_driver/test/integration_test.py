#!/usr/bin/env python
import sys
import time
import unittest

import rospy
import actionlib
import std_msgs.msg
from control_msgs.msg import (
    FollowJointTrajectoryAction,
    FollowJointTrajectoryGoal,
    FollowJointTrajectoryResult,
    JointTolerance)
from ur_dashboard_msgs.msg import SetModeAction, SetModeGoal, RobotMode
from std_srvs.srv import Trigger, TriggerRequest
import tf
from trajectory_msgs.msg import JointTrajectoryPoint
from ur_msgs.srv import SetIO, SetIORequest
from ur_msgs.msg import IOStates

from cartesian_control_msgs.msg import (
    FollowCartesianTrajectoryAction,
    FollowCartesianTrajectoryGoal,
    FollowCartesianTrajectoryResult,
    CartesianTrajectoryPoint)
import geometry_msgs.msg

from controller_manager_msgs.srv import SwitchControllerRequest, SwitchController

PKG = 'ur_robot_driver'
NAME = 'integration_test'
ALL_CONTROLLERS = [
        "scaled_pos_joint_traj_controller",
        "pos_joint_traj_controller",
        "scaled_vel_joint_traj_controller",
        "vel_joint_traj_controller",
        "joint_group_vel_controller",
        "forward_joint_traj_controller",
        "forward_cartesian_traj_controller",
        "twist_controller",
        "pose_based_cartesian_traj_controller",
        "joint_based_cartesian_traj_controller",
        ]


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
        if not self.set_mode_client.wait_for_server(timeout):
            self.fail(
                "Could not reach set_mode action. Make sure that the driver is actually running."
                " Msg: {}".format(err))

        self.trajectory_client = actionlib.SimpleActionClient(
            'follow_joint_trajectory', FollowJointTrajectoryAction)
        if not self.trajectory_client.wait_for_server(timeout):
            self.fail(
                "Could not reach controller action. Make sure that the driver is actually running."
                " Msg: {}".format(err))

        self.cartesian_passthrough_trajectory_client = actionlib.SimpleActionClient(
            'forward_cartesian_trajectory', FollowCartesianTrajectoryAction)
        if not self.cartesian_passthrough_trajectory_client.wait_for_server(timeout):
            self.fail(
                "Could not reach cartesian passthrough controller action. Make sure that the driver is actually running."
                " Msg: {}".format(err))

        self.joint_passthrough_trajectory_client = actionlib.SimpleActionClient(
            'forward_joint_trajectory', FollowJointTrajectoryAction)
        if not self.joint_passthrough_trajectory_client.wait_for_server(timeout):
            self.fail(
                "Could not reach joint passthrough controller action. Make sure that the driver is actually running."
                " Msg: {}".format(err))

        self.cartesian_trajectory_client = actionlib.SimpleActionClient(
            'follow_cartesian_trajectory', FollowCartesianTrajectoryAction)
        if not self.cartesian_trajectory_client.wait_for_server(timeout):
            self.fail(
                "Could not reach cartesian controller action. Make sure that the driver is actually running."
                " Msg: {}".format(err))

        self.set_io_client = rospy.ServiceProxy('/ur_hardware_interface/set_io', SetIO)
        try:
            self.set_io_client.wait_for_service(timeout)
        except rospy.exceptions.ROSException as err:
            self.fail(
                "Could not reach SetIO service. Make sure that the driver is actually running."
                " Msg: {}".format(err))

        self.switch_controllers_client = rospy.ServiceProxy('/controller_manager/switch_controller',
                SwitchController)
        try:
            self.switch_controllers_client.wait_for_service(timeout)
        except rospy.exceptions.ROSException as err:
            self.fail(
                "Could not reach controller switch service. Make sure that the driver is actually running."
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

        self.script_publisher = rospy.Publisher("/ur_hardware_interface/script_command", std_msgs.msg.String, queue_size=1)
        self.tf_listener = tf.TransformListener()
        self.twist_pub = rospy.Publisher("/twist_controller/command", geometry_msgs.msg.Twist, queue_size=1)

    def set_robot_to_mode(self, target_mode):
        goal = SetModeGoal()
        goal.target_robot_mode.mode = RobotMode.RUNNING
        goal.play_program = False # we use headless mode during tests
        # This might be a bug to hunt down. We have to reset the program before calling `resend_robot_program`
        goal.stop_program = True

        self.set_mode_client.send_goal(goal)
        self.set_mode_client.wait_for_result()
        return self.set_mode_client.get_result().success


    def test_joint_trajectory_position_interface(self):
        """Test robot movement"""
        #### Power cycle the robot in order to make sure it is running correctly####
        self.assertTrue(self.set_robot_to_mode(RobotMode.POWER_OFF))
        rospy.sleep(0.5)
        self.assertTrue(self.set_robot_to_mode(RobotMode.RUNNING))
        rospy.sleep(0.5)

        self.send_program_srv.call()
        rospy.sleep(0.5) # TODO properly wait until the controller is running
        self.switch_on_controller("scaled_pos_joint_traj_controller")

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

    def test_cartesian_passthrough(self):
        #### Power cycle the robot in order to make sure it is running correctly####
        self.assertTrue(self.set_robot_to_mode(RobotMode.POWER_OFF))
        rospy.sleep(0.5)
        self.assertTrue(self.set_robot_to_mode(RobotMode.RUNNING))
        rospy.sleep(0.5)

        # Make sure the robot is at a valid start position for our cartesian motions
        self.script_publisher.publish("movej([1, -1.7, -1.7, -1, -1.57, -2])")
        # As we don't have any feedback from that interface, sleep for a while
        rospy.sleep(5)


        self.send_program_srv.call()
        rospy.sleep(0.5) # TODO properly wait until the controller is running

        self.switch_on_controller("forward_cartesian_traj_controller")

        position_list = [geometry_msgs.msg.Vector3(0.4,0.4,0.4)]
        position_list.append(geometry_msgs.msg.Vector3(0.5,0.5,0.5))
        duration_list = [3.0, 6.0]
        goal = FollowCartesianTrajectoryGoal()

        for i, position in enumerate(position_list):
            point = CartesianTrajectoryPoint()
            point.pose = geometry_msgs.msg.Pose(position, geometry_msgs.msg.Quaternion(0,0,0,1))
            point.time_from_start = rospy.Duration(duration_list[i])
            goal.trajectory.points.append(point)
        self.cartesian_passthrough_trajectory_client.send_goal(goal)
        self.cartesian_passthrough_trajectory_client.wait_for_result()
        self.assertEqual(self.cartesian_passthrough_trajectory_client.get_result().error_code,
                         FollowCartesianTrajectoryResult.SUCCESSFUL)
        rospy.loginfo("Received result SUCCESSFUL")

    def test_joint_passthrough(self):
        #### Power cycle the robot in order to make sure it is running correctly####
        self.assertTrue(self.set_robot_to_mode(RobotMode.POWER_OFF))
        rospy.sleep(0.5)
        self.assertTrue(self.set_robot_to_mode(RobotMode.RUNNING))
        rospy.sleep(0.5)

        self.send_program_srv.call()
        rospy.sleep(0.5) # TODO properly wait until the controller is running

        self.switch_on_controller("forward_joint_traj_controller")
        goal = FollowJointTrajectoryGoal()

        goal.trajectory.joint_names = ["elbow_joint", "shoulder_lift_joint", "shoulder_pan_joint",
                                       "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]

        position_list = [[0,-1.57,-1.57,0,0,0]]
        position_list.append([0.2,-1.57,-1.57,0,0,0])
        position_list.append([-0.5,-1.57,-1.2,0,0,0])
        duration_list = [3.0, 7.0, 10.0]

        for i, position in enumerate(position_list):
            point = JointTrajectoryPoint()
            point.positions = position
            point.time_from_start = rospy.Duration(duration_list[i])
            goal.trajectory.points.append(point)
        for i, joint_name in enumerate(goal.trajectory.joint_names):
            goal.goal_tolerance.append(JointTolerance(joint_name, 0.2, 0.2, 0.2))

        goal.goal_time_tolerance = rospy.Duration(0.6)
        self.joint_passthrough_trajectory_client.send_goal(goal)
        self.joint_passthrough_trajectory_client.wait_for_result()

        self.assertEqual(self.joint_passthrough_trajectory_client.get_result().error_code,
                         FollowJointTrajectoryResult.SUCCESSFUL)
        rospy.loginfo("Received result SUCCESSFUL")

    def test_cartesian_trajectory_pose_interface(self):
        #### Power cycle the robot in order to make sure it is running correctly####
        self.assertTrue(self.set_robot_to_mode(RobotMode.POWER_OFF))
        rospy.sleep(0.5)
        self.assertTrue(self.set_robot_to_mode(RobotMode.RUNNING))
        rospy.sleep(0.5)

        # Make sure the robot is at a valid start position for our cartesian motions
        self.script_publisher.publish("movej([1, -1.7, -1.7, -1, -1.57, -2])")
        # As we don't have any feedback from that interface, sleep for a while
        rospy.sleep(5)


        self.send_program_srv.call()
        rospy.sleep(0.5) # TODO properly wait until the controller is running

        self.switch_on_controller("pose_based_cartesian_traj_controller")

        position_list = [geometry_msgs.msg.Vector3(0.4,0.4,0.4)]
        position_list.append(geometry_msgs.msg.Vector3(0.5,0.5,0.5))
        duration_list = [3.0, 6.0]
        goal = FollowCartesianTrajectoryGoal()

        for i, position in enumerate(position_list):
            point = CartesianTrajectoryPoint()
            point.pose = geometry_msgs.msg.Pose(position, geometry_msgs.msg.Quaternion(0,0,0,1))
            point.time_from_start = rospy.Duration(duration_list[i])
            goal.trajectory.points.append(point)
        self.cartesian_trajectory_client.send_goal(goal)
        self.cartesian_trajectory_client.wait_for_result()
        self.assertEqual(self.cartesian_trajectory_client.get_result().error_code,
                         FollowCartesianTrajectoryResult.SUCCESSFUL)
        rospy.loginfo("Received result SUCCESSFUL")

    def test_twist_interface(self):
        #### Power cycle the robot in order to make sure it is running correctly####
        self.assertTrue(self.set_robot_to_mode(RobotMode.POWER_OFF))
        rospy.sleep(0.5)
        self.assertTrue(self.set_robot_to_mode(RobotMode.RUNNING))
        rospy.sleep(0.5)

        # Make sure the robot is at a valid start position for our cartesian motions
        self.script_publisher.publish("movej([1, -1.7, -1.7, -1, -1.57, -2])")
        # As we don't have any feedback from that interface, sleep for a while
        rospy.sleep(5)


        self.send_program_srv.call()
        rospy.sleep(0.5) # TODO properly wait until the controller is running

        self.switch_on_controller("twist_controller")

        # Lookup tcp in base_frame
        (trans_start, rot_start) = self.tf_listener.lookupTransform('base', 'tool0_controller', rospy.Time(0))

        twist = geometry_msgs.msg.Twist()
        twist.linear.x = 0.1
        twist.linear.y = 0.0
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = 0.0

        # publish twist
        self.twist_pub.publish(twist)

        # wait 1 sec
        rospy.sleep(1)

        # stop robot
        twist.linear.x = 0.0
        self.twist_pub.publish(twist)

        (trans_end, rot_end) = self.tf_listener.lookupTransform('base', 'tool0_controller', rospy.Time(0))

        self.assertAlmostEqual(rot_start[0], rot_end[0], delta=3e-6)
        self.assertAlmostEqual(rot_start[1], rot_end[1], delta=1e-6)
        self.assertAlmostEqual(rot_start[2], rot_end[2], delta=1e-6)
        self.assertAlmostEqual(trans_start[1], trans_end[1], delta=1e-6)
        self.assertAlmostEqual(trans_start[2], trans_end[2], delta=1e-6)
        self.assertTrue(trans_end[0] > trans_start[0])


    def switch_on_controller(self, controller_name):
        """Switches on the given controller stopping all other known controllers with best_effort
        strategy."""
        srv = SwitchControllerRequest()
        srv.stop_controllers = ALL_CONTROLLERS
        srv.start_controllers = [controller_name]
        srv.strictness = SwitchControllerRequest.BEST_EFFORT
        result = self.switch_controllers_client(srv)
        self.assertTrue(result.ok)

if __name__ == '__main__':
    import rostest
    rostest.run(PKG, NAME, IntegrationTest, sys.argv)
