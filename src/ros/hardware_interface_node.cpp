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
 * \date    2019-04-11
 *
 */
//----------------------------------------------------------------------
#include <ros/ros.h>
#include <controller_manager/controller_manager.h>

#include <ur_rtde_driver/ros/hardware_interface.h>

int main(int argc, char** argv)
{
  // Set up ROS.
  ros::init(argc, argv, "ur_hardware_interface");
  ros::AsyncSpinner spinner(2);
  spinner.start();

  ros::NodeHandle nh;
  ros::NodeHandle nh_priv("~");

  ur_driver::HardwareInterface hw_interface;

  // Set up timers
  ros::Time timestamp;
  ros::Duration period;
  auto stopwatch_last = std::chrono::steady_clock::now();
  auto stopwatch_now = stopwatch_last;

  hw_interface.init(nh, nh_priv);
  controller_manager::ControllerManager cm(&hw_interface, nh);

  // Get current time and elapsed time since last read
  timestamp = ros::Time::now();
  stopwatch_now = std::chrono::steady_clock::now();
  period.fromSec(std::chrono::duration_cast<std::chrono::duration<double>>(stopwatch_now - stopwatch_last).count());
  stopwatch_last = stopwatch_now;

  ros::Rate control_rate(static_cast<double>(hw_interface.getControlFrequency()));

  // Run as fast as possible
  while (ros::ok())
  {
    // Receive current state from robot
    hw_interface.read(timestamp, period);

    // Get current time and elapsed time since last read
    timestamp = ros::Time::now();
    stopwatch_now = std::chrono::steady_clock::now();
    period.fromSec(std::chrono::duration_cast<std::chrono::duration<double>>(stopwatch_now - stopwatch_last).count());
    stopwatch_last = stopwatch_now;

    cm.update(timestamp, period);

    hw_interface.write(timestamp, period);

    if (!control_rate.sleep())
    {
      ROS_WARN_STREAM("Could not keep cycle rate of " << control_rate.expectedCycleTime().toNSec() / 1000000.0 << "ms");
    }
  }

  spinner.stop();
  ROS_INFO_STREAM_NAMED("hardware_interface", "Shutting down.");
  return 0;
}
