/*
 * ur_driver.cpp
 *
 * Copyright 2015 Thomas Timm Andersen
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "ur_modern_driver/ur_driver.h"
#include "ur_modern_driver/ur_hardware_interface.h"
#include "ur_modern_driver/do_output.h"
#include <string.h>
#include <vector>
#include <mutex>
#include <condition_variable>
#include <thread>
#include <algorithm>
#include <cmath>
#include <chrono>
#include <time.h>

#include "ros/ros.h"
#include <ros/console.h>
#include "sensor_msgs/JointState.h"
#include "geometry_msgs/WrenchStamped.h"
#include "geometry_msgs/PoseStamped.h"
#include "control_msgs/FollowJointTrajectoryAction.h"
#include "actionlib/server/action_server.h"
#include "actionlib/server/server_goal_handle.h"
#include "trajectory_msgs/JointTrajectoryPoint.h"
#include "ur_msgs/SetIO.h"
#include "ur_msgs/SetPayload.h"
#include "ur_msgs/SetPayloadRequest.h"
#include "ur_msgs/SetPayloadResponse.h"
#include "ur_msgs/SetIORequest.h"
#include "ur_msgs/SetIOResponse.h"
#include "ur_msgs/IOStates.h"
#include "ur_msgs/Digital.h"
#include "ur_msgs/Analog.h"
#include "std_msgs/String.h"
#include <controller_manager/controller_manager.h>
#include <realtime_tools/realtime_publisher.h>

/// TF
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>

class RosWrapper {
protected:
	UrDriver robot_;
	std::condition_variable rt_msg_cond_;
	std::condition_variable msg_cond_;
	ros::NodeHandle nh_;
	actionlib::ActionServer<control_msgs::FollowJointTrajectoryAction> as_;
	actionlib::ServerGoalHandle<control_msgs::FollowJointTrajectoryAction> goal_handle_;
	bool has_goal_;
	control_msgs::FollowJointTrajectoryFeedback feedback_;
	control_msgs::FollowJointTrajectoryResult result_;
	ros::Subscriber speed_sub_;
	ros::Subscriber urscript_sub_;
	ros::ServiceServer io_srv_;
	ros::ServiceServer payload_srv_;
	std::thread* rt_publish_thread_;
	std::thread* mb_publish_thread_;
	double io_flag_delay_;
	double max_velocity_;
	std::vector<double> joint_offsets_;
    std::string base_frame_;
    std::string tool_frame_;
	bool use_ros_control_;
	std::thread* ros_control_thread_;
	boost::shared_ptr<ros_control_ur::UrHardwareInterface> hardware_interface_;
	boost::shared_ptr<controller_manager::ControllerManager> controller_manager_;

public:
	RosWrapper(std::string host, int reverse_port) :
			as_(nh_, "follow_joint_trajectory",
					boost::bind(&RosWrapper::goalCB, this, _1),
					boost::bind(&RosWrapper::cancelCB, this, _1), false), robot_(
					rt_msg_cond_, msg_cond_, host, reverse_port, 0.03, 300), io_flag_delay_(0.05), joint_offsets_(
					6, 0.0) {

		std::string joint_prefix = "";
		std::vector<std::string> joint_names;
		char buf[256];

		if (ros::param::get("~prefix", joint_prefix)) {
		    if (joint_prefix.length() > 0) {
    			sprintf(buf, "Setting prefix to %s", joint_prefix.c_str());
	    		print_info(buf);
	        }	
        }
		joint_names.push_back(joint_prefix + "shoulder_pan_joint");
		joint_names.push_back(joint_prefix + "shoulder_lift_joint");
		joint_names.push_back(joint_prefix + "elbow_joint");
		joint_names.push_back(joint_prefix + "wrist_1_joint");
		joint_names.push_back(joint_prefix + "wrist_2_joint");
		joint_names.push_back(joint_prefix + "wrist_3_joint");
		robot_.setJointNames(joint_names);

		use_ros_control_ = false;
		ros::param::get("~use_ros_control", use_ros_control_);

		if (use_ros_control_) {
			hardware_interface_.reset(
					new ros_control_ur::UrHardwareInterface(nh_, &robot_));
			controller_manager_.reset(
					new controller_manager::ControllerManager(
							hardware_interface_.get(), nh_));
			double max_vel_change = 0.12; // equivalent of an acceleration of 15 rad/sec^2
			if (ros::param::get("~max_acceleration", max_vel_change)) {
				max_vel_change = max_vel_change / 125;
			}
			sprintf(buf, "Max acceleration set to: %f [rad/sec²]",
					max_vel_change * 125);
			print_debug(buf);
			hardware_interface_->setMaxVelChange(max_vel_change);
		}
		//Using a very high value in order to not limit execution of trajectories being sent from MoveIt!
		max_velocity_ = 10.;
		if (ros::param::get("~max_velocity", max_velocity_)) {
			sprintf(buf, "Max velocity accepted by ur_driver: %f [rad/s]",
					max_velocity_);
			print_debug(buf);
		}

		//Bounds for SetPayload service
		//Using a very conservative value as it should be set through the parameter server
		double min_payload = 0.;
		double max_payload = 1.;
		if (ros::param::get("~min_payload", min_payload)) {
			sprintf(buf, "Min payload set to: %f [kg]", min_payload);
			print_debug(buf);
		}
		if (ros::param::get("~max_payload", max_payload)) {
			sprintf(buf, "Max payload set to: %f [kg]", max_payload);
			print_debug(buf);
		}
		robot_.setMinPayload(min_payload);
		robot_.setMaxPayload(max_payload);
		sprintf(buf, "Bounds for set_payload service calls: [%f, %f]",
				min_payload, max_payload);
		print_debug(buf);

		double servoj_time = 0.008;
		if (ros::param::get("~servoj_time", servoj_time)) {
			sprintf(buf, "Servoj_time set to: %f [sec]", servoj_time);
			print_debug(buf);
		}
		robot_.setServojTime(servoj_time);

		double servoj_lookahead_time = 0.03;
		if (ros::param::get("~servoj_lookahead_time", servoj_lookahead_time)) {
			sprintf(buf, "Servoj_lookahead_time set to: %f [sec]", servoj_lookahead_time);
			print_debug(buf);
		}
		robot_.setServojLookahead(servoj_lookahead_time);

		double servoj_gain = 300.;
		if (ros::param::get("~servoj_gain", servoj_gain)) {
			sprintf(buf, "Servoj_gain set to: %f [sec]", servoj_gain);
			print_debug(buf);
		}
		robot_.setServojGain(servoj_gain);

        //Base and tool frames
        base_frame_ = joint_prefix + "base_link";
        tool_frame_ =  joint_prefix + "tool0_controller";
        if (ros::param::get("~base_frame", base_frame_)) {
            sprintf(buf, "Base frame set to: %s", base_frame_.c_str());
            print_debug(buf);
        }
        if (ros::param::get("~tool_frame", tool_frame_)) {
            sprintf(buf, "Tool frame set to: %s", tool_frame_.c_str());
            print_debug(buf);
        }

		if (robot_.start()) {
			if (use_ros_control_) {
				ros_control_thread_ = new std::thread(
						boost::bind(&RosWrapper::rosControlLoop, this));
				print_debug(
						"The control thread for this driver has been started");
			} else {
				//start actionserver
				has_goal_ = false;
				as_.start();

				//subscribe to the data topic of interest
				rt_publish_thread_ = new std::thread(
						boost::bind(&RosWrapper::publishRTMsg, this));
				print_debug(
						"The action server for this driver has been started");
			}
			mb_publish_thread_ = new std::thread(
					boost::bind(&RosWrapper::publishMbMsg, this));
			speed_sub_ = nh_.subscribe("ur_driver/joint_speed", 1,
					&RosWrapper::speedInterface, this);
			urscript_sub_ = nh_.subscribe("ur_driver/URScript", 1,
					&RosWrapper::urscriptInterface, this);

			io_srv_ = nh_.advertiseService("ur_driver/set_io",
					&RosWrapper::setIO, this);
			payload_srv_ = nh_.advertiseService("ur_driver/set_payload",
					&RosWrapper::setPayload, this);
		}
	}

	void halt() {
		robot_.halt();
		rt_publish_thread_->join();

	}
private:
	void trajThread(std::vector<double> timestamps,
			std::vector<std::vector<double> > positions,
			std::vector<std::vector<double> > velocities) {

		robot_.doTraj(timestamps, positions, velocities);
		if (has_goal_) {
			result_.error_code = result_.SUCCESSFUL;
			goal_handle_.setSucceeded(result_);
			has_goal_ = false;
		}
	}
	void goalCB(
			actionlib::ServerGoalHandle<
					control_msgs::FollowJointTrajectoryAction> gh) {
		std::string buf;
		print_info("on_goal");
		if (!robot_.sec_interface_->robot_state_->isReady()) {
			result_.error_code = -100; //nothing is defined for this...?

			if (!robot_.sec_interface_->robot_state_->isPowerOnRobot()) {
				result_.error_string =
						"Cannot accept new trajectories: Robot arm is not powered on";
				gh.setRejected(result_, result_.error_string);
				print_error(result_.error_string);
				return;
			}
			if (!robot_.sec_interface_->robot_state_->isRealRobotEnabled()) {
				result_.error_string =
						"Cannot accept new trajectories: Robot is not enabled";
				gh.setRejected(result_, result_.error_string);
				print_error(result_.error_string);
				return;
			}
			result_.error_string =
					"Cannot accept new trajectories. (Debug: Robot mode is "
							+ std::to_string(
									robot_.sec_interface_->robot_state_->getRobotMode())
							+ ")";
			gh.setRejected(result_, result_.error_string);
			print_error(result_.error_string);
			return;
		}
		if (robot_.sec_interface_->robot_state_->isEmergencyStopped()) {
			result_.error_code = -100; //nothing is defined for this...?
			result_.error_string =
					"Cannot accept new trajectories: Robot is emergency stopped";
			gh.setRejected(result_, result_.error_string);
			print_error(result_.error_string);
			return;
		}
		if (robot_.sec_interface_->robot_state_->isProtectiveStopped()) {
			result_.error_code = -100; //nothing is defined for this...?
			result_.error_string =
					"Cannot accept new trajectories: Robot is protective stopped";
			gh.setRejected(result_, result_.error_string);
			print_error(result_.error_string);
			return;
		}

		actionlib::ActionServer<control_msgs::FollowJointTrajectoryAction>::Goal goal =
				*gh.getGoal(); //make a copy that we can modify
		if (has_goal_) {
			print_warning(
					"Received new goal while still executing previous trajectory. Canceling previous trajectory");
			has_goal_ = false;
			robot_.stopTraj();
			result_.error_code = -100; //nothing is defined for this...?
			result_.error_string = "Received another trajectory";
			goal_handle_.setAborted(result_, result_.error_string);
			std::this_thread::sleep_for(std::chrono::milliseconds(250));
		}
		goal_handle_ = gh;
		if (!validateJointNames()) {
			std::string outp_joint_names = "";
			for (unsigned int i = 0; i < goal.trajectory.joint_names.size();
					i++) {
				outp_joint_names += goal.trajectory.joint_names[i] + " ";
			}
			result_.error_code = result_.INVALID_JOINTS;
			result_.error_string =
					"Received a goal with incorrect joint names: "
							+ outp_joint_names;
			gh.setRejected(result_, result_.error_string);
			print_error(result_.error_string);
			return;
		}
		if (!has_positions()) {
			result_.error_code = result_.INVALID_GOAL;
			result_.error_string = "Received a goal without positions";
			gh.setRejected(result_, result_.error_string);
			print_error(result_.error_string);
			return;
		}
        
		if (!has_velocities()) {
			result_.error_code = result_.INVALID_GOAL;
			result_.error_string = "Received a goal without velocities";
			gh.setRejected(result_, result_.error_string);
			print_error(result_.error_string);
			return;
		}

		if (!traj_is_finite()) {
			result_.error_string = "Received a goal with infinities or NaNs";
			result_.error_code = result_.INVALID_GOAL;
			gh.setRejected(result_, result_.error_string);
			print_error(result_.error_string);
			return;
		}

		if (!has_limited_velocities()) {
			result_.error_code = result_.INVALID_GOAL;
			result_.error_string =
					"Received a goal with velocities that are higher than "
							+ std::to_string(max_velocity_);
			gh.setRejected(result_, result_.error_string);
			print_error(result_.error_string);
			return;
		}

		reorder_traj_joints(goal.trajectory);
		
		if (!start_positions_match(goal.trajectory, 0.01)) {
			result_.error_code = result_.INVALID_GOAL;
			result_.error_string = "Goal start doesn't match current pose";
			gh.setRejected(result_, result_.error_string);
			print_error(result_.error_string);
			return;
		}

		std::vector<double> timestamps;
		std::vector<std::vector<double> > positions, velocities;
		if (goal.trajectory.points[0].time_from_start.toSec() != 0.) {
			print_warning(
					"Trajectory's first point should be the current position, with time_from_start set to 0.0 - Inserting point in malformed trajectory");
			timestamps.push_back(0.0);
			positions.push_back(
					robot_.rt_interface_->robot_state_->getQActual());
			velocities.push_back(
					robot_.rt_interface_->robot_state_->getQdActual());
		}
		for (unsigned int i = 0; i < goal.trajectory.points.size(); i++) {
			timestamps.push_back(
					goal.trajectory.points[i].time_from_start.toSec());
			positions.push_back(goal.trajectory.points[i].positions);
			velocities.push_back(goal.trajectory.points[i].velocities);

		}

		goal_handle_.setAccepted();
		has_goal_ = true;
		std::thread(&RosWrapper::trajThread, this, timestamps, positions,
				velocities).detach();
	}

	void cancelCB(
			actionlib::ServerGoalHandle<
					control_msgs::FollowJointTrajectoryAction> gh) {
		// set the action state to preempted
		print_info("on_cancel");
		if (has_goal_) {
			if (gh == goal_handle_) {
				robot_.stopTraj();
				has_goal_ = false;
			}
		}
		result_.error_code = -100; //nothing is defined for this...?
		result_.error_string = "Goal cancelled by client";
		gh.setCanceled(result_);
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
		if (robot_.setPayload(req.payload))
			resp.success = true;
		else
			resp.success = true;
		return resp.success;
	}

	bool validateJointNames() {
		std::vector<std::string> actual_joint_names = robot_.getJointNames();
		actionlib::ActionServer<control_msgs::FollowJointTrajectoryAction>::Goal goal =
				*goal_handle_.getGoal();
		if (goal.trajectory.joint_names.size() != actual_joint_names.size())
			return false;

		for (unsigned int i = 0; i < goal.trajectory.joint_names.size(); i++) {
			unsigned int j;
			for (j = 0; j < actual_joint_names.size(); j++) {
				if (goal.trajectory.joint_names[i] == actual_joint_names[j])
					break;
			}
			if (goal.trajectory.joint_names[i] == actual_joint_names[j]) {
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
				if (traj.points[i].accelerations.size() != 0)
					new_point.accelerations.push_back(
							traj.points[i].accelerations[mapping[j]]);
			}
			new_point.time_from_start = traj.points[i].time_from_start;
			new_traj.push_back(new_point);
		}
		traj.points = new_traj;
	}

	bool has_velocities() {
		actionlib::ActionServer<control_msgs::FollowJointTrajectoryAction>::Goal goal =
				*goal_handle_.getGoal();
		for (unsigned int i = 0; i < goal.trajectory.points.size(); i++) {
			if (goal.trajectory.points[i].positions.size()
					!= goal.trajectory.points[i].velocities.size())
				return false;
		}
		return true;
	}

	bool has_positions() {
		actionlib::ActionServer<control_msgs::FollowJointTrajectoryAction>::Goal goal =
				*goal_handle_.getGoal();
		if (goal.trajectory.points.size() == 0)
			return false;
		for (unsigned int i = 0; i < goal.trajectory.points.size(); i++) {
			if (goal.trajectory.points[i].positions.size()
					!= goal.trajectory.joint_names.size())
				return false;
		}
		return true;
	}

	bool start_positions_match(const trajectory_msgs::JointTrajectory &traj, double eps)
	{
		for (unsigned int i = 0; i < traj.points[0].positions.size(); i++)
		{
			std::vector<double> qActual = robot_.rt_interface_->robot_state_->getQActual();
			if( fabs(traj.points[0].positions[i] - qActual[i]) > eps )
			{
				return false;
			}
		}
		return true;
	}

	bool has_limited_velocities() {
		actionlib::ActionServer<control_msgs::FollowJointTrajectoryAction>::Goal goal =
				*goal_handle_.getGoal();
		for (unsigned int i = 0; i < goal.trajectory.points.size(); i++) {
			for (unsigned int j = 0;
					j < goal.trajectory.points[i].velocities.size(); j++) {
				if (fabs(goal.trajectory.points[i].velocities[j])
						> max_velocity_)
					return false;
			}
		}
		return true;
	}

	bool traj_is_finite() {
		actionlib::ActionServer<control_msgs::FollowJointTrajectoryAction>::Goal goal =
				*goal_handle_.getGoal();
		for (unsigned int i = 0; i < goal.trajectory.points.size(); i++) {
			for (unsigned int j = 0;
					j < goal.trajectory.points[i].velocities.size(); j++) {
				if (!std::isfinite(goal.trajectory.points[i].positions[j]))
					return false;
				if (!std::isfinite(goal.trajectory.points[i].velocities[j]))
					return false;
			}
		}
		return true;
	}

	void speedInterface(const trajectory_msgs::JointTrajectory::Ptr& msg) {
		if (msg->points[0].velocities.size() == 6) {
			double acc = 100;
			if (msg->points[0].accelerations.size() > 0)
				acc = *std::max_element(msg->points[0].accelerations.begin(),
						msg->points[0].accelerations.end());
			robot_.setSpeed(msg->points[0].velocities[0],
					msg->points[0].velocities[1], msg->points[0].velocities[2],
					msg->points[0].velocities[3], msg->points[0].velocities[4],
					msg->points[0].velocities[5], acc);
		}

	}
	void urscriptInterface(const std_msgs::String::ConstPtr& msg) {

		robot_.rt_interface_->addCommandToQueue(msg->data);

	}

	void rosControlLoop() {
		ros::Duration elapsed_time;
		struct timespec last_time, current_time;
		static const double BILLION = 1000000000.0;

		realtime_tools::RealtimePublisher<tf::tfMessage> tf_pub( nh_, "/tf", 1 );
		geometry_msgs::TransformStamped tool_transform;
		tool_transform.header.frame_id = base_frame_;
		tool_transform.child_frame_id = tool_frame_;
		tf_pub.msg_.transforms.push_back( tool_transform );

		realtime_tools::RealtimePublisher<geometry_msgs::TwistStamped> tool_vel_pub( nh_, "tool_velocity", 1 );
		tool_vel_pub.msg_.header.frame_id = base_frame_;


		clock_gettime(CLOCK_MONOTONIC, &last_time);
		while (ros::ok()) {
			std::mutex msg_lock; // The values are locked for reading in the class, so just use a dummy mutex
			std::unique_lock<std::mutex> locker(msg_lock);
			while (!robot_.rt_interface_->robot_state_->getControllerUpdated()) {
				rt_msg_cond_.wait(locker);
			}
			// Input
			hardware_interface_->read();
			robot_.rt_interface_->robot_state_->setControllerUpdated();

			// Control
			clock_gettime(CLOCK_MONOTONIC, &current_time);
			elapsed_time = ros::Duration(current_time.tv_sec - last_time.tv_sec + (current_time.tv_nsec - last_time.tv_nsec)/ BILLION);
			ros::Time ros_time = ros::Time::now();
			controller_manager_->update(ros_time, elapsed_time);
			last_time = current_time;

			// Output
			hardware_interface_->write();

			// Tool vector: Actual Cartesian coordinates of the tool: (x,y,z,rx,ry,rz), where rx, ry and rz is a rotation vector representation of the tool orientation
			std::vector<double> tool_vector_actual = robot_.rt_interface_->robot_state_->getToolVectorActual();

			// Compute rotation angle
			double rx = tool_vector_actual[3];
			double ry = tool_vector_actual[4];
			double rz = tool_vector_actual[5];
			double angle = std::sqrt(std::pow(rx,2) + std::pow(ry,2) + std::pow(rz,2));

			// Broadcast transform
			if( tf_pub.trylock() )
			{			
				tf_pub.msg_.transforms[0].header.stamp = ros_time;
				if (angle < 1e-16) {
					tf_pub.msg_.transforms[0].transform.rotation.x = 0;
					tf_pub.msg_.transforms[0].transform.rotation.y = 0;
					tf_pub.msg_.transforms[0].transform.rotation.z = 0;
					tf_pub.msg_.transforms[0].transform.rotation.w = 1;
				} else {
					tf_pub.msg_.transforms[0].transform.rotation.x = (rx/angle) * std::sin(angle*0.5);
					tf_pub.msg_.transforms[0].transform.rotation.y = (ry/angle) * std::sin(angle*0.5);
					tf_pub.msg_.transforms[0].transform.rotation.z = (rz/angle) * std::sin(angle*0.5);
					tf_pub.msg_.transforms[0].transform.rotation.w = std::cos(angle*0.5);
				}
				tf_pub.msg_.transforms[0].transform.translation.x = tool_vector_actual[0];
				tf_pub.msg_.transforms[0].transform.translation.y = tool_vector_actual[1];
				tf_pub.msg_.transforms[0].transform.translation.z = tool_vector_actual[2];

				tf_pub.unlockAndPublish();
			}

			//Publish tool velocity
			std::vector<double> tcp_speed = robot_.rt_interface_->robot_state_->getTcpSpeedActual();

			if( tool_vel_pub.trylock() )
			{			
				tool_vel_pub.msg_.header.stamp = ros_time;
				tool_vel_pub.msg_.twist.linear.x = tcp_speed[0];
				tool_vel_pub.msg_.twist.linear.y = tcp_speed[1];
				tool_vel_pub.msg_.twist.linear.z = tcp_speed[2];
				tool_vel_pub.msg_.twist.angular.x = tcp_speed[3];
				tool_vel_pub.msg_.twist.angular.y = tcp_speed[4];
				tool_vel_pub.msg_.twist.angular.z = tcp_speed[5];

				tool_vel_pub.unlockAndPublish();
			}

		}
	}

	void publishRTMsg() {
		ros::Publisher joint_pub = nh_.advertise<sensor_msgs::JointState>(
				"joint_states", 1);
		ros::Publisher wrench_pub = nh_.advertise<geometry_msgs::WrenchStamped>(
				"wrench", 1);
        ros::Publisher tool_vel_pub = nh_.advertise<geometry_msgs::TwistStamped>("tool_velocity", 1);
        static tf::TransformBroadcaster br;
		while (ros::ok()) {
			sensor_msgs::JointState joint_msg;
			joint_msg.name = robot_.getJointNames();
			geometry_msgs::WrenchStamped wrench_msg;
            geometry_msgs::PoseStamped tool_pose_msg;
			std::mutex msg_lock; // The values are locked for reading in the class, so just use a dummy mutex
			std::unique_lock<std::mutex> locker(msg_lock);
			while (!robot_.rt_interface_->robot_state_->getDataPublished()) {
				rt_msg_cond_.wait(locker);
			}
			joint_msg.header.stamp = ros::Time::now();
			joint_msg.position =
					robot_.rt_interface_->robot_state_->getQActual();
			for (unsigned int i = 0; i < joint_msg.position.size(); i++) {
				joint_msg.position[i] += joint_offsets_[i];
			}
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

            // Tool vector: Actual Cartesian coordinates of the tool: (x,y,z,rx,ry,rz), where rx, ry and rz is a rotation vector representation of the tool orientation
            std::vector<double> tool_vector_actual = robot_.rt_interface_->robot_state_->getToolVectorActual();

            //Create quaternion
            tf::Quaternion quat;
            double rx = tool_vector_actual[3];
            double ry = tool_vector_actual[4];
            double rz = tool_vector_actual[5];
            double angle = std::sqrt(std::pow(rx,2) + std::pow(ry,2) + std::pow(rz,2));
            if (angle < 1e-16) {
                quat.setValue(0, 0, 0, 1);
            } else {
                quat.setRotation(tf::Vector3(rx/angle, ry/angle, rz/angle), angle);
            }

            //Create and broadcast transform
            tf::Transform transform;
            transform.setOrigin(tf::Vector3(tool_vector_actual[0], tool_vector_actual[1], tool_vector_actual[2]));
            transform.setRotation(quat);
            br.sendTransform(tf::StampedTransform(transform, joint_msg.header.stamp, base_frame_, tool_frame_));

            //Publish tool velocity
            std::vector<double> tcp_speed =
                    robot_.rt_interface_->robot_state_->getTcpSpeedActual();
            geometry_msgs::TwistStamped tool_twist;
            tool_twist.header.frame_id = base_frame_;
            tool_twist.header.stamp = joint_msg.header.stamp;
            tool_twist.twist.linear.x = tcp_speed[0];
            tool_twist.twist.linear.y = tcp_speed[1];
            tool_twist.twist.linear.z = tcp_speed[2];
            tool_twist.twist.angular.x = tcp_speed[3];
            tool_twist.twist.angular.y = tcp_speed[4];
            tool_twist.twist.angular.z = tcp_speed[5];
            tool_vel_pub.publish(tool_twist);

			robot_.rt_interface_->robot_state_->setDataPublished();
		}
	}

	void publishMbMsg() {
		bool warned = false;
		ros::Publisher io_pub = nh_.advertise<ur_msgs::IOStates>(
				"ur_driver/io_states", 1);

		while (ros::ok()) {
			ur_msgs::IOStates io_msg;
			std::mutex msg_lock; // The values are locked for reading in the class, so just use a dummy mutex
			std::unique_lock<std::mutex> locker(msg_lock);
			while (!robot_.sec_interface_->robot_state_->getNewDataAvailable()) {
				msg_cond_.wait(locker);
			}
			int i_max = 10;
			if (robot_.sec_interface_->robot_state_->getVersion() > 3.0)
				i_max = 18; // From version 3.0, there are up to 18 inputs and outputs
			for (unsigned int i = 0; i < i_max; i++) {
				ur_msgs::Digital digi;
				digi.pin = i;
				digi.state =
						((robot_.sec_interface_->robot_state_->getDigitalInputBits()
								& (1 << i)) >> i);
				io_msg.digital_in_states.push_back(digi);
				digi.state =
						((robot_.sec_interface_->robot_state_->getDigitalOutputBits()
								& (1 << i)) >> i);
				io_msg.digital_out_states.push_back(digi);
			}
			ur_msgs::Analog ana;
			ana.pin = 0;
			ana.state = robot_.sec_interface_->robot_state_->getAnalogInput0();
			io_msg.analog_in_states.push_back(ana);
			ana.pin = 1;
			ana.state = robot_.sec_interface_->robot_state_->getAnalogInput1();
			io_msg.analog_in_states.push_back(ana);

			ana.pin = 0;
			ana.state = robot_.sec_interface_->robot_state_->getAnalogOutput0();
			io_msg.analog_out_states.push_back(ana);
			ana.pin = 1;
			ana.state = robot_.sec_interface_->robot_state_->getAnalogOutput1();
			io_msg.analog_out_states.push_back(ana);
			io_pub.publish(io_msg);

			if (robot_.sec_interface_->robot_state_->isEmergencyStopped()
					or robot_.sec_interface_->robot_state_->isProtectiveStopped()) {
				if (robot_.sec_interface_->robot_state_->isEmergencyStopped()
						and !warned) {
					print_error("Emergency stop pressed!");
				} else if (robot_.sec_interface_->robot_state_->isProtectiveStopped()
						and !warned) {
					print_error("Robot is protective stopped!");
				}
				if (has_goal_) {
					print_error("Aborting trajectory");
					robot_.stopTraj();
					result_.error_code = result_.SUCCESSFUL;
					result_.error_string = "Robot was halted";
					goal_handle_.setAborted(result_, result_.error_string);
					has_goal_ = false;
				}
				warned = true;
			} else
				warned = false;

			robot_.sec_interface_->robot_state_->finishedReading();

		}
	}

};

int main(int argc, char **argv) {
	bool use_sim_time = false;
	std::string host;
	int reverse_port = 50001;

	ros::init(argc, argv, "ur_driver");
	ros::NodeHandle nh;
	if (ros::param::get("use_sim_time", use_sim_time)) {
		print_warning("use_sim_time is set!!");
	}
	if (!(ros::param::get("~robot_ip_address", host))) {
		if (argc > 1) {
			print_warning(
					"Please set the parameter robot_ip_address instead of giving it as a command line argument. This method is DEPRECATED");
			host = argv[1];
		} else {
			print_fatal(
					"Could not get robot ip. Please supply it as command line parameter or on the parameter server as robot_ip");
			exit(1);
		}

	}
	if ((ros::param::get("~reverse_port", reverse_port))) {
		if((reverse_port <= 0) or (reverse_port >= 65535)) {
			print_warning("Reverse port value is not valid (Use number between 1 and 65534. Using default value of 50001");
			reverse_port = 50001;
		}
	} else
		reverse_port = 50001;

	RosWrapper interface(host, reverse_port);

	ros::AsyncSpinner spinner(3);
	spinner.start();

	ros::waitForShutdown();

	interface.halt();

	exit(0);
}
