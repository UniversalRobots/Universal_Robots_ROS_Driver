/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2015, PickNik LLC
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
 *   * Neither the name of PickNik LLC nor the names of its
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
   Desc:   Helper ros_control hardware interface that loads configurations
*/

#include "ur_robot_driver/ros/hardware_interface.h"

namespace ur_driver
{
void HardwareInterface::registerJointLimits(ros::NodeHandle& robot_hw_nh,
                                            const hardware_interface::JointHandle& joint_handle_position,
                                            const hardware_interface::JointHandle& joint_handle_velocity,
                                            std::size_t joint_id)
{
  // Default values
  joint_position_lower_limits_[joint_id] = -std::numeric_limits<double>::max();
  joint_position_upper_limits_[joint_id] = std::numeric_limits<double>::max();
  joint_velocity_limits_[joint_id] = std::numeric_limits<double>::max();

  // Limits datastructures
  joint_limits_interface::JointLimits joint_limits;     // Position
  joint_limits_interface::SoftJointLimits soft_limits;  // Soft Position
  bool has_joint_limits = false;
  bool has_soft_limits = false;

  // Get limits from URDF
  if (urdf_model_ == nullptr)
  {
    ROS_WARN_STREAM("No URDF model loaded, unable to get joint limits");
    return;
  }

  // Get limits from URDF
  urdf::JointConstSharedPtr urdf_joint = urdf_model_->getJoint(joint_names_[joint_id]);

  // Get main joint limits
  if (urdf_joint == nullptr)
  {
    ROS_ERROR_STREAM("URDF joint not found " << joint_names_[joint_id]);
    return;
  }

  // Get limits from URDF
  if (joint_limits_interface::getJointLimits(urdf_joint, joint_limits))
  {
    has_joint_limits = true;
    ROS_DEBUG_STREAM("Joint " << joint_names_[joint_id] << " has URDF position limits [" << joint_limits.min_position
                              << ", " << joint_limits.max_position << "]");
    if (joint_limits.has_velocity_limits)
      ROS_DEBUG_STREAM("Joint " << joint_names_[joint_id] << " has URDF velocity limit [" << joint_limits.max_velocity
                                << "]");
  }
  else
  {
    if (urdf_joint->type != urdf::Joint::CONTINUOUS)
      ROS_WARN_STREAM("No URDF limits are configured for joint " << joint_names_[joint_id]);
  }

  // Get limits from ROS param
  if (joint_limits_interface::getJointLimits(joint_names_[joint_id], robot_hw_nh, joint_limits))
  {
    has_joint_limits = true;
    ROS_DEBUG_STREAM("Joint " << joint_names_[joint_id] << " has rosparam position limits ["
                              << joint_limits.min_position << ", " << joint_limits.max_position << "]");
    if (joint_limits.has_velocity_limits)
      ROS_DEBUG_STREAM("Joint " << joint_names_[joint_id] << " has rosparam velocity limit ["
                                << joint_limits.max_velocity << "]");
  }  // the else debug message provided internally by joint_limits_interface

  // Get soft limits from URDF
  if (joint_limits_interface::getSoftJointLimits(urdf_joint, soft_limits))
  {
    has_soft_limits = true;
    ROS_DEBUG_STREAM("Joint " << joint_names_[joint_id] << " has soft joint limits.");
  }
  else
  {
    ROS_DEBUG_STREAM("Joint " << joint_names_[joint_id]
                              << " does not have soft joint "
                                 "limits");
  }

  // Quit we we haven't found any limits in URDF or rosparam server
  if (!has_joint_limits)
  {
    return;
  }

  // Copy position limits if available
  if (joint_limits.has_position_limits)
  {
    // Slighly reduce the joint limits to prevent floating point errors
    joint_limits.min_position += std::numeric_limits<double>::epsilon();
    joint_limits.max_position -= std::numeric_limits<double>::epsilon();

    joint_position_lower_limits_[joint_id] = joint_limits.min_position;
    joint_position_upper_limits_[joint_id] = joint_limits.max_position;
  }

  // Copy velocity limits if available
  if (joint_limits.has_velocity_limits)
  {
    joint_velocity_limits_[joint_id] = joint_limits.max_velocity;
  }

  if (has_soft_limits)  // Use soft limits
  {
    ROS_DEBUG_STREAM("Using soft saturation limits");
    const joint_limits_interface::PositionJointSoftLimitsHandle soft_handle_position(joint_handle_position,
                                                                                     joint_limits, soft_limits);
    pos_jnt_soft_interface_.registerHandle(soft_handle_position);
    const joint_limits_interface::VelocityJointSoftLimitsHandle soft_handle_velocity(joint_handle_velocity,
                                                                                     joint_limits, soft_limits);
    vel_jnt_soft_interface_.registerHandle(soft_handle_velocity);
  }
  else  // Use saturation limits
  {
    ROS_DEBUG_STREAM("Using saturation limits (not soft limits)");

    const joint_limits_interface::PositionJointSaturationHandle sat_handle_position(joint_handle_position,
                                                                                    joint_limits);
    pos_jnt_sat_interface_.registerHandle(sat_handle_position);

    const joint_limits_interface::VelocityJointSaturationHandle sat_handle_velocity(joint_handle_velocity,
                                                                                    joint_limits);
    vel_jnt_sat_interface_.registerHandle(sat_handle_velocity);
  }
}

void HardwareInterface::loadURDF(ros::NodeHandle& nh, std::string param_name)
{
  std::string urdf_string;
  urdf_model_ = new urdf::Model();

  // search and wait for robot_description on param server
  while (urdf_string.empty() && ros::ok())
  {
    std::string search_param_name;
    if (nh.searchParam(param_name, search_param_name))
    {
      ROS_INFO_STREAM("Waiting for model URDF on the ROS param server at location: " << nh.getNamespace()
                                                                                     << search_param_name);
      nh.getParam(search_param_name, urdf_string);
    }

    usleep(100000);
  }

  if (!urdf_model_->initString(urdf_string))
    ROS_ERROR_STREAM("Unable to load URDF model");
  else
    ROS_DEBUG_STREAM("Received URDF from param server");
}
}  // namespace ur_driver
