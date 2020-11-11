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
 * \date    2019-10-21
 *
 */
//----------------------------------------------------------------------

#include <ros/ros.h>
#include <ur_robot_driver/dashboard_client_ros.h>

int main(int argc, char** argv)
{
  // Set up ROS.
  ros::init(argc, argv, "dashboard_client");
  ros::NodeHandle priv_nh("~");

  // The IP address under which the robot is reachable.
  std::string robot_ip = priv_nh.param<std::string>("robot_ip", "192.168.56.101");

  ur_driver::DashboardClientROS client(priv_nh, robot_ip);

  ros::spin();
  return 0;
}
