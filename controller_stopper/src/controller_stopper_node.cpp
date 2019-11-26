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
 * \date    2019-06-12
 *
 */
//----------------------------------------------------------------------

#include <ros/ros.h>

#include <controller_stopper/controller_stopper.h>

int main(int argc, char** argv)
{
  // Set up ROS.
  ros::init(argc, argv, "controller_stopper_node");
  ros::NodeHandle nh;
  ros::NodeHandle priv_nh("");

  ControllerStopper stopper(nh);

  ros::spin();
  return 0;
}
