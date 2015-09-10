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
#include <algorithm>

#include "sensor_msgs/JointState.h"
#include "geometry_msgs/WrenchStamped.h"
#include "control_msgs/FollowJointTrajectoryAction.h"
#include "actionlib/server/simple_action_server.h"
#include "trajectory_msgs/JointTrajectoryPoint.h"
#include "ur_msgs/SetIO.h"
#include "ur_msgs/SetPayload.h"
#include "ur_msgs/SetPayloadRequest.h"
#include "ur_msgs/SetPayloadResponse.h"
#include "ur_msgs/SetIORequest.h"
#include "ur_msgs/SetIOResponse.h"

class RosWrapper {
protected:
	UrDriver robot_;
	std::condition_variable msg_cond_;
	ros::NodeHandle nh_;
	actionlib::SimpleActionServer<control_msgs::FollowJointTrajectoryAction> as_;
	actionlib::SimpleActionServer<control_msgs::FollowJointTrajectoryAction>::Goal goal_;
	control_msgs::FollowJointTrajectoryFeedback feedback_;
	control_msgs::FollowJointTrajectoryResult result_;
	ros::Subscriber speedSub_;
	ros::ServiceServer ioSrv_;
	ros::ServiceServer payloadSrv_;
	std::thread* rt_publish_thread_;
	double io_flag_delay_;

public:
	RosWrapper(std::string host) :
			as_(nh_, "follow_joint_trajectory", false), robot_(msg_cond_, host), io_flag_delay_(0.05) {

		std::string joint_prefix = "";
		std::vector<std::string> joint_names;

		if (ros::param::get("~prefix", joint_prefix)) {
			ROS_INFO("Setting prefix to %s", joint_prefix.c_str());
		}
		joint_names.push_back(joint_prefix + "shoulder_pan_joint");
		joint_names.push_back(joint_prefix + "shoulder_lift_joint");
		joint_names.push_back(joint_prefix + "elbow_joint");
		joint_names.push_back(joint_prefix + "wrist_1_joint");
		joint_names.push_back(joint_prefix + "wrist_2_joint");
		joint_names.push_back(joint_prefix + "wrist_3_joint");
		robot_.setJointNames(joint_names);

		robot_.start();

		//register the goal and feedback callbacks
		as_.registerGoalCallback(boost::bind(&RosWrapper::goalCB, this));
		as_.registerPreemptCallback(boost::bind(&RosWrapper::preemptCB, this));

		as_.start();

		//subscribe to the data topic of interest
		speedSub_ = nh_.subscribe("joint_speed", 1, &RosWrapper::speedInterface,
				this);

		ioSrv_ = nh_.advertiseService("ur_driver/set_io", &RosWrapper::setIO,
				this);
		payloadSrv_ = nh_.advertiseService("ur_driver/set_payload",
				&RosWrapper::setPayload, this);

		rt_publish_thread_ = new std::thread(
				boost::bind(&RosWrapper::publishRTMsg, this));
		ROS_INFO("The action server for this driver has been started");

	}

	void halt() {
		robot_.halt();
		rt_publish_thread_->join();

	}
private:
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
			as_.setAborted(result_,
					("Received a goal with incorrect joint names: %s", outp_joint_names.c_str()));
		}

		reorder_traj_joints(goal_.trajectory);
		std::vector<double> timestamps;
		std::vector<std::vector<double> > positions, velocities;
		for (unsigned int i = 0; i < goal_.trajectory.points.size(); i++) {
			timestamps.push_back(
					goal_.trajectory.points[i].time_from_start.toSec());
			positions.push_back(goal_.trajectory.points[i].positions);
			velocities.push_back(goal_.trajectory.points[i].velocities);

		}
		robot_.addTraj(timestamps, positions, velocities);

		ros::Duration(timestamps.back()).sleep();
		result_.error_code = result_.SUCCESSFUL;
		as_.setSucceeded(result_);

	}

	void preemptCB() {
		ROS_INFO("on_cancel");
		// set the action state to preempted
		robot_.stopTraj();
		as_.setPreempted();
	}

	bool setIO(ur_msgs::SetIORequest& req, ur_msgs::SetIOResponse& resp) {
		resp.success = true;
		//if (req.fun == ur_msgs::SetIO::Request::FUN_SET_DIGITAL_OUT) {
		if (req.fun == 1) {
			robot_.setDigitalOut(req.pin, req.state > 0.0 ? true : false);
		} else if (req.fun == 2) {
			//} else if (req.fun == ur_msgs::SetIO::Request::FUN_SET_FLAG) {
			robot_.setFlag(req.pin, req.state > 0.0 ? true : false);
			//According to urdriver.py, set_flag will fail if called to rapidly in succession
			ros::Duration(io_flag_delay_).sleep();
		} else if (req.fun == 3) {
			//} else if (req.fun == ur_msgs::SetIO::Request::FUN_SET_ANALOG_OUT) {
			robot_.setAnalogOut(req.pin, req.state);
		} else if (req.fun == 4) {
			//} else if (req.fun == ur_msgs::SetIO::Request::FUN_SET_TOOL_VOLTAGE) {
			robot_.setToolVoltage((int) req.state);
		} else {
			resp.success = false;
		}
		return resp.success;
	}

	bool setPayload(ur_msgs::SetPayloadRequest& req,
			ur_msgs::SetPayloadResponse& resp) {
		robot_.setPayloaf(req.payload);
		resp.success = true;
		return true;
	}

	bool validateJointNames() {
		std::vector<std::string> actual_joint_names = robot_.getJointNames();
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

	void reorder_traj_joints(trajectory_msgs::JointTrajectory& traj) {
		/* Reorders trajectory - destructive */
		std::vector<std::string> actual_joint_names = robot_.getJointNames();
		std::vector<unsigned int> mapping;
		mapping.resize(actual_joint_names.size(), actual_joint_names.size());
		for (unsigned int i = 0; i < traj.joint_names.size(); i++) {
			for (unsigned int j = 0; j < actual_joint_names.size(); j++) {
				if (traj.joint_names[i] == actual_joint_names[j])
					mapping[j] = i;
			}
		}
		traj.joint_names = actual_joint_names;
		std::vector<trajectory_msgs::JointTrajectoryPoint> new_traj;
		for (unsigned int i = 0; i < traj.points.size(); i++) {
			trajectory_msgs::JointTrajectoryPoint new_point;
			for (unsigned int j = 0; j < traj.points[i].positions.size(); j++) {
				new_point.positions.push_back(
						traj.points[i].positions[mapping[j]]);
				new_point.velocities.push_back(
						traj.points[i].velocities[mapping[j]]);
				new_point.accelerations.push_back(
						traj.points[i].accelerations[mapping[j]]);
			}
			new_point.time_from_start = traj.points[i].time_from_start;
			new_traj.push_back(new_point);
		}
		traj.points = new_traj;
	}

	void speedInterface(const trajectory_msgs::JointTrajectory::Ptr& msg) {
		reorder_traj_joints(*msg);
		double acc = *std::max_element(msg->points[0].accelerations.begin(),
				msg->points[0].accelerations.end());
		robot_.setSpeed(msg->points[0].velocities[0],
				msg->points[0].velocities[1], msg->points[0].velocities[2],
				msg->points[0].velocities[3], msg->points[0].velocities[4],
				msg->points[0].velocities[5], acc);

	}

	void publishRTMsg() {
		ros::Publisher joint_pub = nh_.advertise<sensor_msgs::JointState>(
				"/joint_states", 1);
		ros::Publisher wrench_pub = nh_.advertise<geometry_msgs::WrenchStamped>(
				"/wrench", 1);
		while (ros::ok()) {
			sensor_msgs::JointState joint_msg;
			joint_msg.name = robot_.getJointNames();
			geometry_msgs::WrenchStamped wrench_msg;
			std::mutex msg_lock; // The values are locked for reading in the class, so just use a dummy mutex
			std::unique_lock<std::mutex> locker(msg_lock);
			while (!robot_.rt_interface_->robot_state_->getNewDataAvailable()) {
				msg_cond_.wait(locker);
			}
			joint_msg.header.stamp = ros::Time::now();
			joint_msg.position =
					robot_.rt_interface_->robot_state_->getQActual();
			joint_msg.velocity =
					robot_.rt_interface_->robot_state_->getQdActual();
			joint_msg.effort = robot_.rt_interface_->robot_state_->getIActual();
			joint_pub.publish(joint_msg);
			std::vector<double> tcp_force =
					robot_.rt_interface_->robot_state_->getTcpForce();
			wrench_msg.header.stamp = joint_msg.header.stamp;
			wrench_msg.wrench.force.x = tcp_force[0];
			wrench_msg.wrench.force.y = tcp_force[1];
			wrench_msg.wrench.force.z = tcp_force[2];
			wrench_msg.wrench.torque.x = tcp_force[3];
			wrench_msg.wrench.torque.y = tcp_force[4];
			wrench_msg.wrench.torque.z = tcp_force[5];
			wrench_pub.publish(wrench_msg);

			robot_.rt_interface_->robot_state_->finishedReading();

		}
	}

};

int main(int argc, char **argv) {
	bool use_sim_time = false;
	std::string host;

	ros::init(argc, argv, "ur_driver");
	ros::NodeHandle nh;
	if (ros::param::get("use_sim_time", use_sim_time)) {
		ROS_WARN("use_sim_time is set!!");
	}
	if (argc > 1) {
		host = argv[1];
	} else if (!(ros::param::get("~robot_ip", host))) {
		ROS_FATAL(
				"Could not get robot ip. Please supply it as command line parameter or on the parameter server as robot_ip");
		exit(1);
	}

	RosWrapper interface(host);

	ros::spin();
	interface.halt();

	exit(0);
}
