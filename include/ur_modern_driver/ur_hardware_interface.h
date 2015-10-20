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

#ifndef UR_ROS_CONTROL_UR_HARDWARE_INTERFACE_H
#define UR_ROS_CONTROL_UR_HARDWARE_INTERFACE_H

#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/force_torque_sensor_interface.h>
#include <hardware_interface/robot_hw.h>
#include <controller_manager/controller_manager.h>
#include <boost/scoped_ptr.hpp>
#include <ros/ros.h>
#include <math.h>
#include "do_output.h"
#include "ur_driver.h"

namespace ros_control_ur {

// For simulation only - determines how fast a trajectory is followed
static const double POSITION_STEP_FACTOR = 1;
static const double VELOCITY_STEP_FACTOR = 1;

/// \brief Hardware interface for a robot
class UrHardwareInterface: public hardware_interface::RobotHW {
public:

	/**
	 * \brief Constructor
	 * \param nh - Node handle for topics.
	 */
	UrHardwareInterface(ros::NodeHandle& nh, UrDriver* robot);

	/// \brief Initialize the hardware interface
	virtual void init();

	/// \brief Read the state from the robot hardware.
	virtual void read();

	/// \brief write the command to the robot hardware.
	virtual void write();

	void setMaxVelChange(double inp);

	bool canSwitch(
			const std::list<hardware_interface::ControllerInfo> &start_list,
			const std::list<hardware_interface::ControllerInfo> &stop_list) const;
	void doSwitch(const std::list<hardware_interface::ControllerInfo>&start_list,
			const std::list<hardware_interface::ControllerInfo>&stop_list);

protected:

	// Startup and shutdown of the internal node inside a roscpp program
	ros::NodeHandle nh_;

	// Interfaces
	hardware_interface::JointStateInterface joint_state_interface_;
	hardware_interface::ForceTorqueSensorInterface force_torque_interface_;
	hardware_interface::PositionJointInterface position_joint_interface_;
	hardware_interface::VelocityJointInterface velocity_joint_interface_;
	bool velocity_interface_running_;
	bool position_interface_running_;
	// Shared memory
	std::vector<std::string> joint_names_;
	std::vector<double> joint_position_;
	std::vector<double> joint_velocity_;
	std::vector<double> joint_effort_;
	std::vector<double> joint_position_command_;
	std::vector<double> joint_velocity_command_;
	std::vector<double> prev_joint_velocity_command_;
		std::size_t num_joints_;
	double robot_force_[3] = { 0., 0., 0. };
	double robot_torque_[3] = { 0., 0., 0. };

	double max_vel_change_;

	// Robot API
	UrDriver* robot_;

};
// class

}// namespace

#endif
