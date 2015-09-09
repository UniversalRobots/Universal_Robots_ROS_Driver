/*
 * ur_driver.cpp
 *
 * ----------------------------------------------------------------------------
 * "THE BEER-WARE LICENSE" (Revision 42):
 * <thomas.timm.dk@gmail.com> wrote this file.  As long as you retain this notice you
 * can do whatever you want with this stuff. If we meet some day, and you think
 * this stuff is worth it, you can buy me a beer in return.   Thomas Timm Andersen
 * ----------------------------------------------------------------------------
 */

#include "ros/ros.h"
#include <ros/console.h>
#include "ur_modern_driver/ur_driver.h"
#include <string.h>
#include <vector>
#include <mutex>
#include <condition_variable>
#include <thread>

#include "sensor_msgs/JointState.h"
#include "geometry_msgs/WrenchStamped.h"
#include "control_msgs/FollowJointTrajectoryAction.h"
#include "actionlib/server/simple_action_server.h"
#include "trajectory_msgs/JointTrajectoryPoint.h"

std::condition_variable g_msg_cond;

class URTrajectoryFollower {
protected:
	ros::NodeHandle nh_;
	actionlib::SimpleActionServer<control_msgs::FollowJointTrajectoryAction> as_;
	actionlib::SimpleActionServer<control_msgs::FollowJointTrajectoryAction>::Goal goal_;
	control_msgs::FollowJointTrajectoryFeedback feedback_;
	control_msgs::FollowJointTrajectoryResult result_;
	ros::Subscriber sub_;
	UrDriver* robot_;
public:
	URTrajectoryFollower(UrDriver* robot) :
			as_(nh_, "follow_joint_trajectory", false) {
		robot_ = robot;
		//register the goal and feeback callbacks
		as_.registerGoalCallback(
				boost::bind(&URTrajectoryFollower::goalCB, this));
		as_.registerPreemptCallback(
				boost::bind(&URTrajectoryFollower::preemptCB, this));

		//subscribe to the data topic of interest
		as_.start();
		ROS_INFO("The action server for this driver has been started");
	}
	void goalCB() {
		ROS_INFO("on_goal");

		actionlib::SimpleActionServer<control_msgs::FollowJointTrajectoryAction>::GoalConstPtr goal =
				as_.acceptNewGoal();
		goal_ = *goal; //make a copy that we can modify
		if (!validateJointNames()) {
			std::string outp_joint_names = "";
			for (unsigned int i = 0; i < goal_.trajectory.joint_names.size();
					i++) {
				outp_joint_names += goal_.trajectory.joint_names[i] + " ";
			}
			ROS_ERROR("Received a goal with incorrect joint names: %s",
					outp_joint_names.c_str());
			result_.error_code = result_.INVALID_JOINTS;
			/*result_.error_string =
					"Received a goal with incorrect joint names: %s", outp_joint_names.c_str(); */
			as_.setAborted(result_, ("Received a goal with incorrect joint names: %s", outp_joint_names.c_str()));
		}

		reorder_traj_joints();
		std::vector<double> timestamps;
		std::vector<std::vector<double> > positions, velocities;
		for (unsigned int i = 0; i < goal_.trajectory.points.size(); i++) {
			timestamps.push_back(goal_.trajectory.points[i].time_from_start.toSec());
			positions.push_back(goal_.trajectory.points[i].positions);
			velocities.push_back(goal_.trajectory.points[i].velocities);

		}
		robot_->addTraj(timestamps, positions, velocities);

		//sleep?
		result_.error_code = result_.SUCCESSFUL;
		as_.setSucceeded(result_);

	}

	void preemptCB() {
		ROS_INFO("on_cancel");
		// set the action state to preempted
		robot_->stopTraj();
		as_.setPreempted();
	}
private:
	bool validateJointNames() {
		std::vector<std::string> actual_joint_names = robot_->getJointNames();
		if (goal_.trajectory.joint_names.size() != actual_joint_names.size())
			return false;

		for (unsigned int i = 0; i < goal_.trajectory.joint_names.size(); i++) {
			unsigned int j;
			for (j = 0; j < actual_joint_names.size(); j++) {
				if (goal_.trajectory.joint_names[i] == actual_joint_names[j])
					break;
			}
			if (goal_.trajectory.joint_names[i] == actual_joint_names[j]) {
				actual_joint_names.erase(actual_joint_names.begin() + j);
			} else {
				return false;
			}
		}

		return true;
	}

	void reorder_traj_joints() {
		std::vector<std::string> actual_joint_names = robot_->getJointNames();
		std::vector<unsigned int> mapping;
		mapping.resize(actual_joint_names.size(), actual_joint_names.size());
		for (unsigned int i = 0; i < goal_.trajectory.joint_names.size(); i++) {
			for (unsigned int j = 0; j < actual_joint_names.size(); j++) {
				if (goal_.trajectory.joint_names[i] == actual_joint_names[j])
					mapping[j] = i;
			}
		}
		goal_.trajectory.joint_names = actual_joint_names;
		std::vector<trajectory_msgs::JointTrajectoryPoint> new_traj;
		for (unsigned int i = 0; i < goal_.trajectory.points.size(); i++) {
			trajectory_msgs::JointTrajectoryPoint new_point;
			for (unsigned int j = 0;
					j < goal_.trajectory.points[i].positions.size(); j++) {
				new_point.positions.push_back(
						goal_.trajectory.points[i].positions[mapping[j]]);
				new_point.velocities.push_back(
						goal_.trajectory.points[i].velocities[mapping[j]]);
				new_point.accelerations.push_back(
						goal_.trajectory.points[i].accelerations[mapping[j]]);
			}
			new_point.time_from_start =
					goal_.trajectory.points[i].time_from_start;
			new_traj.push_back(new_point);
		}
		goal_.trajectory.points = new_traj;
	}

};

void publishRTMsg(UrDriver robot) {
	ros::NodeHandle nh_rt;
	ros::Publisher joint_pub = nh_rt.advertise<sensor_msgs::JointState>(
			"/joint_states", 1);
	ros::Publisher wrench_pub = nh_rt.advertise<geometry_msgs::WrenchStamped>(
			"/wrench", 1);
	while (ros::ok()) {
		sensor_msgs::JointState joint_msg;
		joint_msg.name = robot.getJointNames();
		geometry_msgs::WrenchStamped wrench_msg;
		std::mutex msg_lock; // The values are locked for reading in the class, so just use a dummy mutex
		std::unique_lock<std::mutex> locker(msg_lock);
		while (!robot.rt_interface_->robot_state_->getNewDataAvailable()) {
			g_msg_cond.wait(locker);
		}
		joint_msg.header.stamp = ros::Time::now();
		joint_msg.position = robot.rt_interface_->robot_state_->getQActual();
		joint_msg.velocity = robot.rt_interface_->robot_state_->getQdActual();
		joint_msg.effort = robot.rt_interface_->robot_state_->getIActual();
		joint_pub.publish(joint_msg);
		std::vector<double> tcp_force =
				robot.rt_interface_->robot_state_->getTcpForce();
		wrench_msg.header.stamp = joint_msg.header.stamp;
		wrench_msg.wrench.force.x = tcp_force[0];
		wrench_msg.wrench.force.y = tcp_force[1];
		wrench_msg.wrench.force.z = tcp_force[2];
		wrench_msg.wrench.torque.x = tcp_force[3];
		wrench_msg.wrench.torque.y = tcp_force[4];
		wrench_msg.wrench.torque.z = tcp_force[5];
		wrench_pub.publish(wrench_msg);

		robot.rt_interface_->robot_state_->finishedReading();

	}
}

int main(int argc, char **argv) {
	bool use_sim_time = false;
	std::string joint_prefix = "";
	std::string host;
	std::vector<std::string> joint_names;

	ros::init(argc, argv, "ur_driver");
	ros::NodeHandle nh;
	if (ros::param::get("use_sim_time", use_sim_time)) {
		ROS_WARN("use_sim_time is set!!");
	}
	if (ros::param::get("~prefix", joint_prefix)) {
		ROS_INFO("Setting prefix to %s", joint_prefix.c_str());
	}
	joint_names.push_back(joint_prefix + "shoulder_pan_joint");
	joint_names.push_back(joint_prefix + "shoulder_lift_joint");
	joint_names.push_back(joint_prefix + "elbow_joint");
	joint_names.push_back(joint_prefix + "wrist_1_joint");
	joint_names.push_back(joint_prefix + "wrist_2_joint");
	joint_names.push_back(joint_prefix + "wrist_3_joint");

	if (argc > 1) {
		host = argv[1];
	} else if (!(ros::param::get("~robot_ip", host))) {
		ROS_FATAL(
				"Could not get robot ip. Please supply it as command line parameter or on the parameter server as robot_ip");
		exit(1);
	}
	UrDriver robot(g_msg_cond, host, joint_names);

	robot.start();

	std::thread rt_publish_thread(publishRTMsg, robot);
	URTrajectoryFollower action_server(&robot);

	ros::spin();
	robot.halt();
	exit(0);
}
