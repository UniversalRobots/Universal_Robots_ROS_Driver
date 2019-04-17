// this is for emacs file handling -*- mode: c++; indent-tabs-mode: nil -*-

// -- BEGIN LICENSE BLOCK ----------------------------------------------
// -- END LICENSE BLOCK ------------------------------------------------

//----------------------------------------------------------------------
/*!\file
 *
 * \author  Felix Mauch mauch@fzi.de
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

PLUGINLIB_EXPORT_CLASS(position_controllers::ScaledJointTrajectoryController, controller_interface::ControllerBase)
