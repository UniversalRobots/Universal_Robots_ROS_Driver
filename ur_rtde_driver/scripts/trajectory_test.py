#!/usr/bin/env python

import rospy
import actionlib
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectoryPoint

def main(position_list, duration_list):
    rospy.init_node('trajectory_testing_client')
    client = actionlib.SimpleActionClient('/scaled_pos_traj_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
    client.wait_for_server()
    goal = FollowJointTrajectoryGoal()

    goal.trajectory.joint_names = ["elbow_joint", "shoulder_lift_joint", "shoulder_pan_joint", "wrist_1_joint",
            "wrist_2_joint", "wrist_3_joint"]

    for i in range(len(position_list)):
        point = JointTrajectoryPoint()
        point.positions = position_list[i]
        point.time_from_start = rospy.Duration(duration_list[i])
        goal.trajectory.points.append(point)

    client.send_goal(goal)
    client.wait_for_result()

    return client.get_result().error_code == 0




if __name__ == '__main__':
    position_list = [[0.0 for i in range(6)]]
    position_list.append([-0.5 for i in range(6)])
    position_list.append([-1.0 for i in range(6)])
    duration_list = [6.0, 9.0, 12.0]

    print(main(position_list, duration_list))
