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
 * \author  Felix Mauch mauch@fzi.de
 * \date    2019-06-13
 *
 */
//----------------------------------------------------------------------

#include <ur_rtde_driver/ros/tcp_accuracy_checker.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace ur_driver
{
TcpAccuracyChecker::TcpAccuracyChecker(const std::string& frame_a, const std::string& frame_b,
                                       const double desired_accuracy)
  : frame_a_(frame_a), frame_b_(frame_b), desired_accuracy_(desired_accuracy), actual_accuracy_(0)
{
  worker_thread_.reset(new std::thread(&TcpAccuracyChecker::asyncCheck, this, 2, 10));
}

bool TcpAccuracyChecker::checkAccuracy()
{
  try
  {
    tf_listener_.waitForTransform(frame_a_, frame_b_, ros::Time(0), ros::Duration(120));
    tf_listener_.lookupTransform(frame_a_, frame_b_, ros::Time(0), transform_);
  }
  catch (tf::TransformException ex)
  {
    ROS_ERROR("TF lookup error error: %s", ex.what());
  }

  actual_accuracy_ = transform_.getOrigin().length();
  return actual_accuracy_ <= desired_accuracy_;
}

void TcpAccuracyChecker::asyncCheck(const uint32_t interval, const uint32_t num_checks)
{
  ros::Rate r(1.0 / interval);
  for (size_t i = 0; i < num_checks; ++i)
  {
    if (!checkAccuracy())
    {
      ROS_ERROR_STREAM("Desired accuracy of "
                       << desired_accuracy_ << " between " << frame_a_ << " and " << frame_b_
                       << " was violated. Actual difference: " << actual_accuracy_ << std::endl
                       << "This is critical! Your robot might not be at the expected position." << std::endl
                       << "Use the ur_calibration tool to extract the correct calibration from the robot and pass that "
                          "into the description. See [TODO Link to documentation] for details.");
    }
    r.sleep();
  }
}
}  // namespace ur_driver
