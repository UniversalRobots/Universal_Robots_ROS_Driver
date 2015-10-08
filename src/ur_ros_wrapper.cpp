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
#include <time.h>

#include "ros/ros.h"
#include <ros/console.h>
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
#include "ur_msgs/IOStates.h"
#include "ur_msgs/Digital.h"
#include "ur_msgs/Analog.h"
#include "std_msgs/String.h"
#include <controller_manager/controller_manager.h>
#include <diagnostic_updater/diagnostic_updater.h>
#include <diagnostic_updater/publisher.h>

enum RobotMode {
    ROBOT_MODE_DISCONNECTED,
    ROBOT_MODE_CONFIRM_SAFETY,
    ROBOT_MODE_BOOTING,
    ROBOT_MODE_POWER_OFF,
    ROBOT_MODE_POWER_ON,
    ROBOT_MODE_IDLE,
    ROBOT_MODE_BACKDRIVE,
    ROBOT_MODE_RUNNING,
    ROBOT_MODE_UPDATING_FIRMWARE
};

enum SafetyMode {
    SAFETY_MODE_NORMAL,
    SAFETY_MODE_REDUCED,
    SAFETY_MODE_PROTECTIVE_STOP,
    SAFETY_MODE_RECOVERY,
    SAFETY_MODE_SAFEGUARD_STOP,
    SAFETY_MODE_SYSTEM_EMERGENCY_STOP,
    SAFETY_MODE_ROBOT_EMERGENCY_STOP,
    SAFETY_MODE_VIOLATION,
    SAFETY_MODE_FAULT
};

struct DriverStatus{
    RobotMode robot_mode;
    SafetyMode safety_mode;
};

class RosWrapper {
protected:
	UrDriver robot_;
	std::condition_variable rt_msg_cond_;
	std::condition_variable msg_cond_;
	ros::NodeHandle nh_;
	actionlib::SimpleActionServer<control_msgs::FollowJointTrajectoryAction> as_;
	actionlib::SimpleActionServer<control_msgs::FollowJointTrajectoryAction>::Goal goal_;
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
	bool use_ros_control_;
	std::thread* ros_control_thread_;
	boost::shared_ptr<ros_control_ur::UrHardwareInterface> hardware_interface_;
	boost::shared_ptr<controller_manager::ControllerManager> controller_manager_;
    diagnostic_updater::Updater updater_;
    DriverStatus driver_status_;

public:
	RosWrapper(std::string host) :
			as_(nh_, "follow_joint_trajectory", false), robot_(rt_msg_cond_,
					msg_cond_, host), io_flag_delay_(0.05), joint_offsets_(6,
					0.0) {

		std::string joint_prefix = "";
		std::vector<std::string> joint_names;
		char buf[256];

        updater_.setHardwareID("ur_driver");
        updater_.add("Status updater", this, &RosWrapper::driverDiagnostic);

		if (ros::param::get("~prefix", joint_prefix)) {
			sprintf(buf, "Setting prefix to %s", joint_prefix.c_str());
			print_info(buf);
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

		double servoj_time = 0.016;
		if (ros::param::get("~servoj_time", servoj_time)) {
			sprintf(buf, "Servoj_time set to: %f [sec]", servoj_time);
			print_debug(buf);
		}
		robot_.setServojTime(servoj_time);

		if (robot_.start()) {
			if (use_ros_control_) {
				ros_control_thread_ = new std::thread(
						boost::bind(&RosWrapper::rosControlLoop, this));
				print_debug(
						"The control thread for this driver has been started");
			} else {
				//register the goal and feedback callbacks
				as_.registerGoalCallback(
						boost::bind(&RosWrapper::goalCB, this));
				as_.registerPreemptCallback(
						boost::bind(&RosWrapper::preemptCB, this));

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
    void driverDiagnostic(diagnostic_updater::DiagnosticStatusWrapper &stat){

        if (driver_status_.robot_mode == ROBOT_MODE_RUNNING &&
                (driver_status_.safety_mode == SAFETY_MODE_NORMAL || driver_status_.safety_mode == SAFETY_MODE_REDUCED)){
            //Robot is running and in normal safety mode
            stat.summary(diagnostic_msgs::DiagnosticStatus::OK, "Driver OK");
        }
        else if (driver_status_.robot_mode == ROBOT_MODE_RUNNING &&
                 (driver_status_.safety_mode != SAFETY_MODE_NORMAL || driver_status_.safety_mode != SAFETY_MODE_REDUCED)){
            //Robot is running and in some safety stop
            stat.summary(diagnostic_msgs::DiagnosticStatus::WARN, "Driver WARNING");
        }
        else {
            //Robot is not running
            stat.summary(diagnostic_msgs::DiagnosticStatus::ERROR, "Driver ERROR");
        }
        stat.add("Robot mode", driver_status_.robot_mode);
        stat.add("Safety mode", driver_status_.safety_mode);
    }

	void goalCB() {
		print_info("on_goal");

		actionlib::SimpleActionServer<control_msgs::FollowJointTrajectoryAction>::GoalConstPtr goal =
				as_.acceptNewGoal();
		goal_ = *goal; //make a copy that we can modify
		if (!validateJointNames()) {
			std::string outp_joint_names = "";
			for (unsigned int i = 0; i < goal_.trajectory.joint_names.size();
					i++) {
				outp_joint_names += goal_.trajectory.joint_names[i] + " ";
			}
			result_.error_code = result_.INVALID_JOINTS;
			result_.error_string =
					"Received a goal with incorrect joint names: "
							+ outp_joint_names;
			as_.setAborted(result_, result_.error_string);
			print_error(result_.error_string);
		}

		if (!has_positions()) {
			result_.error_code = result_.INVALID_GOAL;
			result_.error_string = "Received a goal without positions";
			as_.setAborted(result_, result_.error_string);
			print_error(result_.error_string);
		}

		if (!has_velocities()) {
			result_.error_code = result_.INVALID_GOAL;
			result_.error_string = "Received a goal without velocities";
			as_.setAborted(result_, result_.error_string);
			print_error(result_.error_string);
		}

		if (!traj_is_finite()) {
			result_.error_string = "Received a goal with infinites or NaNs";
			result_.error_code = result_.INVALID_GOAL;
			as_.setAborted(result_, result_.error_string);
			print_error(result_.error_string);
		}

		if (!has_limited_velocities()) {
			result_.error_code = result_.INVALID_GOAL;
			result_.error_string =
					"Received a goal with velocities that are higher than %f", max_velocity_;
			as_.setAborted(result_, result_.error_string);
			print_error(result_.error_string);
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
		robot_.doTraj(timestamps, positions, velocities);
		result_.error_code = result_.SUCCESSFUL;
		as_.setSucceeded(result_);
	}

	void preemptCB() {
		print_info("on_cancel");
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
		if (robot_.setPayload(req.payload))
			resp.success = true;
		else
			resp.success = true;
		return resp.success;
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
		for (unsigned int i = 0; i < goal_.trajectory.points.size(); i++) {
			if (goal_.trajectory.points[i].positions.size()
					!= goal_.trajectory.points[i].velocities.size())
				return false;
		}
		return true;
	}

	bool has_positions() {
		if (goal_.trajectory.points.size() == 0)
			return false;
		for (unsigned int i = 0; i < goal_.trajectory.points.size(); i++) {
			if (goal_.trajectory.points[i].positions.size()
					!= goal_.trajectory.joint_names.size())
				return false;
		}
		return true;
	}

	bool has_limited_velocities() {
		for (unsigned int i = 0; i < goal_.trajectory.points.size(); i++) {
			for (unsigned int j = 0;
					j < goal_.trajectory.points[i].velocities.size(); j++) {
				if (fabs(goal_.trajectory.points[i].velocities[j])
						> max_velocity_)
					return false;
			}
		}
		return true;
	}

	bool traj_is_finite() {
		for (unsigned int i = 0; i < goal_.trajectory.points.size(); i++) {
			for (unsigned int j = 0;
					j < goal_.trajectory.points[i].velocities.size(); j++) {
				if (!std::isfinite(goal_.trajectory.points[i].positions[j]))
					return false;
				if (!std::isfinite(goal_.trajectory.points[i].velocities[j]))
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

		clock_gettime(CLOCK_MONOTONIC, &last_time);
		while (ros::ok()) {
			std::mutex msg_lock; // The values are locked for reading in the class, so just use a dummy mutex
			std::unique_lock<std::mutex> locker(msg_lock);
			while (!robot_.rt_interface_->robot_state_->getControllerUpdated()) {
				rt_msg_cond_.wait(locker);
			}
			clock_gettime(CLOCK_MONOTONIC, &current_time);
			elapsed_time = ros::Duration(
					current_time.tv_sec - last_time.tv_sec
							+ (current_time.tv_nsec - last_time.tv_nsec)
									/ BILLION);
			last_time = current_time;
			// Input
			hardware_interface_->read();
			robot_.rt_interface_->robot_state_->setControllerUpdated();
			// Control
			controller_manager_->update(
					ros::Time(current_time.tv_sec, current_time.tv_nsec),
					elapsed_time);

			// Output
			hardware_interface_->write();
		}
	}

	void publishRTMsg() {
		ros::Publisher joint_pub = nh_.advertise<sensor_msgs::JointState>(
				"joint_states", 1);
		ros::Publisher wrench_pub = nh_.advertise<geometry_msgs::WrenchStamped>(
				"wrench", 1);
		while (ros::ok()) {
			sensor_msgs::JointState joint_msg;
			joint_msg.name = robot_.getJointNames();
			geometry_msgs::WrenchStamped wrench_msg;
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

            //Update diagnostics
            driver_status_.robot_mode = static_cast<RobotMode>( robot_.rt_interface_->robot_state_->getRobotMode() );
            driver_status_.safety_mode = static_cast<SafetyMode>( robot_.rt_interface_->robot_state_->getSafety_mode() );
            updater_.update();

			robot_.rt_interface_->robot_state_->setDataPublished();

		}
	}

	void publishMbMsg() {
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

			robot_.sec_interface_->robot_state_->finishedReading();

		}
	}

};

int main(int argc, char **argv) {
	bool use_sim_time = false;
	std::string host;

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

	RosWrapper interface(host);

	ros::AsyncSpinner spinner(3);
	spinner.start();

	ros::waitForShutdown();

	interface.halt();

	exit(0);
}
