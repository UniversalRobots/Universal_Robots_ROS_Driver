// this is for emacs file handling -*- mode: c++; indent-tabs-mode: nil -*-

// -- BEGIN LICENSE BLOCK ----------------------------------------------
// Copyright 2019 FZI Forschungszentrum Informatik
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
// -- END LICENSE BLOCK ------------------------------------------------

//----------------------------------------------------------------------
/*!\file
 *
 * \author  Felix Exner exner@fzi.de
 * \date    2019-04-18
 *
 */
//----------------------------------------------------------------------

#include "ur_controllers/scaled_joint_trajectory_controller.h"
#include "ur_controllers/scaled_joint_command_interface.h"

#include <pluginlib/class_list_macros.hpp>
#include <trajectory_interface/quintic_spline_segment.h>

namespace position_controllers
{
typedef ur_controllers::ScaledJointTrajectoryController<trajectory_interface::QuinticSplineSegment<double>,
                                                        ur_controllers::ScaledPositionJointInterface>
    ScaledJointTrajectoryController;
}

namespace velocity_controllers
{
typedef ur_controllers::ScaledJointTrajectoryController<trajectory_interface::QuinticSplineSegment<double>,
                                                        ur_controllers::ScaledVelocityJointInterface>
    ScaledJointTrajectoryController;
}

PLUGINLIB_EXPORT_CLASS(position_controllers::ScaledJointTrajectoryController, controller_interface::ControllerBase)
PLUGINLIB_EXPORT_CLASS(velocity_controllers::ScaledJointTrajectoryController, controller_interface::ControllerBase)
