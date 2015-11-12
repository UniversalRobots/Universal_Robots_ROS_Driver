/*
 * ur_hardware_control_loop.cpp
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

/* Based on original source from University of Colorado, Boulder. License copied below. */

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
 *********************************************************************

 Author: Dave Coleman
 */

#include <ur_modern_driver/ur_hardware_interface.h>

namespace ros_control_ur {

UrHardwareInterface::UrHardwareInterface(ros::NodeHandle& nh, UrDriver* robot) :
		nh_(nh), robot_(robot) {
	// Initialize shared memory and interfaces here
	init(); // this implementation loads from rosparam

	max_vel_change_ = 0.12; // equivalent of an acceleration of 15 rad/sec^2

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
	joint_position_command_.resize(num_joints_);
	joint_velocity_command_.resize(num_joints_);
	prev_joint_velocity_command_.resize(num_joints_);

	// Initialize controller
	for (std::size_t i = 0; i < num_joints_; ++i) {
		ROS_DEBUG_STREAM_NAMED("ur_hardware_interface",
				"Loading joint name: " << joint_names_[i]);

		// Create joint state interface
		joint_state_interface_.registerHandle(
				hardware_interface::JointStateHandle(joint_names_[i],
						&joint_position_[i], &joint_velocity_[i],
						&joint_effort_[i]));

		// Create position joint interface
		position_joint_interface_.registerHandle(
				hardware_interface::JointHandle(
						joint_state_interface_.getHandle(joint_names_[i]),
						&joint_position_command_[i]));

		// Create velocity joint interface
		velocity_joint_interface_.registerHandle(
				hardware_interface::JointHandle(
						joint_state_interface_.getHandle(joint_names_[i]),
						&joint_velocity_command_[i]));
		prev_joint_velocity_command_[i] = 0.;
	}

	// Create force torque interface
	force_torque_interface_.registerHandle(
			hardware_interface::ForceTorqueSensorHandle("wrench", "",
					robot_force_, robot_torque_));

	registerInterface(&joint_state_interface_); // From RobotHW base class.
	registerInterface(&position_joint_interface_); // From RobotHW base class.
	registerInterface(&velocity_joint_interface_); // From RobotHW base class.
	registerInterface(&force_torque_interface_); // From RobotHW base class.
	velocity_interface_running_ = false;
	position_interface_running_ = false;
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

void UrHardwareInterface::setMaxVelChange(double inp) {
	max_vel_change_ = inp;
}

void UrHardwareInterface::write() {
	if (velocity_interface_running_) {
		std::vector<double> cmd;
		//do some rate limiting
		cmd.resize(joint_velocity_command_.size());
		for (unsigned int i = 0; i < joint_velocity_command_.size(); i++) {
			cmd[i] = joint_velocity_command_[i];
			if (cmd[i] > prev_joint_velocity_command_[i] + max_vel_change_) {
				cmd[i] = prev_joint_velocity_command_[i] + max_vel_change_;
			} else if (cmd[i]
					< prev_joint_velocity_command_[i] - max_vel_change_) {
				cmd[i] = prev_joint_velocity_command_[i] - max_vel_change_;
			}
			prev_joint_velocity_command_[i] = cmd[i];
		}
		robot_->setSpeed(cmd[0], cmd[1], cmd[2], cmd[3], cmd[4], cmd[5],  max_vel_change_*125);
	} else if (position_interface_running_) {
		robot_->servoj(joint_position_command_);
	}
}

bool UrHardwareInterface::canSwitch(
		const std::list<hardware_interface::ControllerInfo> &start_list,
		const std::list<hardware_interface::ControllerInfo> &stop_list) const {
	for (std::list<hardware_interface::ControllerInfo>::const_iterator controller_it =
			start_list.begin(); controller_it != start_list.end();
			++controller_it) {
		if (controller_it->hardware_interface
				== "hardware_interface::VelocityJointInterface") {
			if (velocity_interface_running_) {
				ROS_ERROR(
						"%s: An interface of that type (%s) is already running",
						controller_it->name.c_str(),
						controller_it->hardware_interface.c_str());
				return false;
			}
			if (position_interface_running_) {
				bool error = true;
				for (std::list<hardware_interface::ControllerInfo>::const_iterator stop_controller_it =
						stop_list.begin();
						stop_controller_it != stop_list.end();
						++stop_controller_it) {
					if (stop_controller_it->hardware_interface
							== "hardware_interface::PositionJointInterface") {
						error = false;
						break;
					}
				}
				if (error) {
					ROS_ERROR(
							"%s (type %s) can not be run simultaneously with a PositionJointInterface",
							controller_it->name.c_str(),
							controller_it->hardware_interface.c_str());
					return false;
				}
			}
		} else if (controller_it->hardware_interface
				== "hardware_interface::PositionJointInterface") {
			if (position_interface_running_) {
				ROS_ERROR(
						"%s: An interface of that type (%s) is already running",
						controller_it->name.c_str(),
						controller_it->hardware_interface.c_str());
				return false;
			}
			if (velocity_interface_running_) {
				bool error = true;
				for (std::list<hardware_interface::ControllerInfo>::const_iterator stop_controller_it =
						stop_list.begin();
						stop_controller_it != stop_list.end();
						++stop_controller_it) {
					if (stop_controller_it->hardware_interface
							== "hardware_interface::VelocityJointInterface") {
						error = false;
						break;
					}
				}
				if (error) {
					ROS_ERROR(
							"%s (type %s) can not be run simultaneously with a VelocityJointInterface",
							controller_it->name.c_str(),
							controller_it->hardware_interface.c_str());
					return false;
				}
			}
		}
	}

// we can always stop a controller
	return true;
}

void UrHardwareInterface::doSwitch(
		const std::list<hardware_interface::ControllerInfo>& start_list,
		const std::list<hardware_interface::ControllerInfo>& stop_list) {
	for (std::list<hardware_interface::ControllerInfo>::const_iterator controller_it =
			stop_list.begin(); controller_it != stop_list.end();
			++controller_it) {
		if (controller_it->hardware_interface
				== "hardware_interface::VelocityJointInterface") {
			velocity_interface_running_ = false;
			ROS_DEBUG("Stopping velocity interface");
		}
		if (controller_it->hardware_interface
				== "hardware_interface::PositionJointInterface") {
			position_interface_running_ = false;
			std::vector<double> tmp;
			robot_->closeServo(tmp);
			ROS_DEBUG("Stopping position interface");
		}
	}
	for (std::list<hardware_interface::ControllerInfo>::const_iterator controller_it =
			start_list.begin(); controller_it != start_list.end();
			++controller_it) {
		if (controller_it->hardware_interface
				== "hardware_interface::VelocityJointInterface") {
			velocity_interface_running_ = true;
			ROS_DEBUG("Starting velocity interface");
		}
		if (controller_it->hardware_interface
				== "hardware_interface::PositionJointInterface") {
			position_interface_running_ = true;
			robot_->uploadProg();
			ROS_DEBUG("Starting position interface");
		}
	}

}

} // namespace

