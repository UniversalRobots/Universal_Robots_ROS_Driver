/*
 * ur_hardware_control_loop.cpp
 *
 * ----------------------------------------------------------------------------
 * "THE BEER-WARE LICENSE" (Revision 42):
 * <thomas.timm.dk@gmail.com> wrote this file.  As long as you retain this notice you
 * can do whatever you want with this stuff. If we meet some day, and you think
 * this stuff is worth it, you can buy me a beer in return.   Thomas Timm Andersen
 * ----------------------------------------------------------------------------
 */

/* Based on original source from University of Colorado, Boulder. License copied below. Feel free to offer Dave a beer as well if you meet him. */

/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2015, University of Colorado, Boulder
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Univ of CO, Boulder nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Dave Coleman
 Desc:   Example ros_control hardware interface that performs a perfect control loop for simulation
 */

#include <ur_modern_driver/ur_hardware_interface.h>

namespace ros_control_ur {

UrHardwareInterface::UrHardwareInterface(ros::NodeHandle& nh, UrDriver* robot) :
		nh_(nh), robot_(robot) {
	// Initialize shared memory and interfaces here
	init(); // this implementation loads from rosparam

	ROS_INFO_NAMED("ur_hardware_interface", "Loaded ur_hardware_interface.");
}

void UrHardwareInterface::init() {
	ROS_INFO_STREAM_NAMED("ur_hardware_interface",
			"Reading rosparams from namespace: " << nh_.getNamespace());

	// Get joint names
	nh_.getParam("hardware_interface/joints", joint_names_);
	if (joint_names_.size() == 0) {
		ROS_FATAL_STREAM_NAMED("ur_hardware_interface",
				"No joints found on parameter server for controller, did you load the proper yaml file?" << " Namespace: " << nh_.getNamespace());
		exit(-1);
	}
	num_joints_ = joint_names_.size();

	// Resize vectors
	joint_position_.resize(num_joints_);
	joint_velocity_.resize(num_joints_);
	joint_effort_.resize(num_joints_);
	joint_velocity_command_.resize(num_joints_);

	// Initialize controller
	for (std::size_t i = 0; i < num_joints_; ++i) {
		ROS_DEBUG_STREAM_NAMED("ur_hardware_interface",
				"Loading joint name: " << joint_names_[i]);

		// Create joint state interface
		joint_state_interface_.registerHandle(
				hardware_interface::JointStateHandle(joint_names_[i],
						&joint_position_[i], &joint_velocity_[i],
						&joint_effort_[i]));

		// Create velocity joint interface
		velocity_joint_interface_.registerHandle(
				hardware_interface::JointHandle(
						joint_state_interface_.getHandle(joint_names_[i]),
						&joint_velocity_command_[i]));
	}

	// Create joint state interface
	force_torque_interface_.registerHandle(
			hardware_interface::ForceTorqueSensorHandle("wrench", "",
					robot_force_, robot_torque_));

	registerInterface(&joint_state_interface_); // From RobotHW base class.
	registerInterface(&velocity_joint_interface_); // From RobotHW base class.
	registerInterface(&force_torque_interface_); // From RobotHW base class.
}

void UrHardwareInterface::read() {
	std::vector<double> pos, vel, current, tcp;
	pos = robot_->rt_interface_->robot_state_->getQActual();
	vel = robot_->rt_interface_->robot_state_->getQdActual();
	current = robot_->rt_interface_->robot_state_->getIActual();
	tcp = robot_->rt_interface_->robot_state_->getTcpForce();
	for (std::size_t i = 0; i < num_joints_; ++i) {
		joint_position_[i] = pos[i];
		joint_velocity_[i] = vel[i];
		joint_effort_[i] = current[i];
	}
	for (std::size_t i = 0; i < 3; ++i) {
		robot_force_[i] = tcp[i];
		robot_torque_[i] = tcp[i + 3];
	}

}

void UrHardwareInterface::write() {
	robot_->setSpeed(joint_velocity_command_[0],joint_velocity_command_[1],joint_velocity_command_[2],joint_velocity_command_[3],joint_velocity_command_[4],joint_velocity_command_[5],100);
}

} // namespace
