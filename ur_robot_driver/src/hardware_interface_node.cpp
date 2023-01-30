// this is for emacs file handling -*- mode: c++; indent-tabs-mode: nil -*-

// -- BEGIN LICENSE BLOCK ----------------------------------------------
// Copyright 2019 FZI Forschungszentrum Informatik
// Created on behalf of Universal Robots A/S
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
 * \date    2019-04-11
 *
 */
//----------------------------------------------------------------------
#include <ros/ros.h>
#include <controller_manager/controller_manager.h>

#include <csignal>
#include <ur_robot_driver/hardware_interface.h>
#include <ur_robot_driver/urcl_log_handler.h>

std::unique_ptr<ur_driver::HardwareInterface> g_hw_interface;

void signalHandler(int signum)
{
  std::cout << "Interrupt signal (" << signum << ") received.\n";

  g_hw_interface.reset();
  // cleanup and close up stuff here
  // terminate program

  exit(signum);
}

bool setFiFoScheduling(pthread_t& thread, const int priority)
{
  struct sched_param params;
  params.sched_priority = priority;
  int ret = pthread_setschedparam(thread, SCHED_FIFO, &params);
  if (ret != 0)
  {
    switch (ret)
    {
      case EPERM:
      {
        ROS_ERROR_STREAM("Your system/user seems not to be setup for FIFO scheduling. We recommend using a lowlatency "
                         "kernel with FIFO scheduling. See "
                         "https://github.com/UniversalRobots/Universal_Robots_ROS_Driver/blob/master/ur_robot_driver/"
                         "doc/real_time.md for details.");
        break;
      }
      default:

      {
        ROS_ERROR_STREAM("Unsuccessful in setting thread to FIFO scheduling with priority " << priority << ". "
                                                                                            << strerror(ret));
      }
    }
  }
  // Now verify the change in thread priority
  int policy = 0;
  ret = pthread_getschedparam(thread, &policy, &params);
  if (ret != 0)
  {
    ROS_ERROR("Couldn't retrieve scheduling parameters");
    return false;
  }

  // Check the correct policy was applied
  if (policy != SCHED_FIFO)
  {
    ROS_ERROR("Scheduling is NOT SCHED_FIFO!");
    return false;
  }
  else
  {
    ROS_INFO_STREAM("SCHED_FIFO OK, priority " << params.sched_priority);
    if (params.sched_priority != priority)
    {
      return false;
    }
  }
  return true;
}

int main(int argc, char** argv)
{
  // Set up ROS.
  ros::init(argc, argv, "ur_hardware_interface");
  ros::AsyncSpinner spinner(2);
  spinner.start();

  ros::NodeHandle nh;
  ros::NodeHandle nh_priv("~");

  // register signal SIGINT and signal handler
  signal(SIGINT, signalHandler);

  ur_driver::registerUrclLogHandler();

  pthread_t this_thread = pthread_self();
  const int max_thread_priority = sched_get_priority_max(SCHED_FIFO);
  bool is_fifo = setFiFoScheduling(this_thread, max_thread_priority);
  ROS_INFO_STREAM("The driver's control thread uses " << (is_fifo ? "" : "NO") << " FIFO scheduling");

  // Set up timers
  ros::Time timestamp;
  ros::Duration period;
  auto stopwatch_last = std::chrono::steady_clock::now();
  auto stopwatch_now = stopwatch_last;

  g_hw_interface.reset(new ur_driver::HardwareInterface);

  if (!g_hw_interface->init(nh, nh_priv))
  {
    ROS_ERROR_STREAM("Could not correctly initialize robot. Exiting");
    exit(1);
  }
  ROS_DEBUG_STREAM("initialized hw interface");
  controller_manager::ControllerManager cm(g_hw_interface.get(), nh);

  // Get current time and elapsed time since last read
  timestamp = ros::Time::now();
  stopwatch_now = std::chrono::steady_clock::now();
  period.fromSec(std::chrono::duration_cast<std::chrono::duration<double>>(stopwatch_now - stopwatch_last).count());
  stopwatch_last = stopwatch_now;

  double expected_cycle_time = 1.0 / (static_cast<double>(g_hw_interface->getControlFrequency()));

  // Run as fast as possible
  while (ros::ok())
  {
    // Receive current state from robot
    g_hw_interface->read(timestamp, period);

    // Get current time and elapsed time since last read
    timestamp = ros::Time::now();
    stopwatch_now = std::chrono::steady_clock::now();
    period.fromSec(std::chrono::duration_cast<std::chrono::duration<double>>(stopwatch_now - stopwatch_last).count());
    stopwatch_last = stopwatch_now;

    cm.update(timestamp, period, g_hw_interface->shouldResetControllers());

    g_hw_interface->write(timestamp, period);
    // if (!control_rate.sleep())
    if (period.toSec() > expected_cycle_time)
    {
      // ROS_WARN_STREAM("Could not keep cycle rate of " << expected_cycle_time * 1000 << "ms");
      // ROS_WARN_STREAM("Actual cycle time:" << period.toNSec() / 1000000.0 << "ms");
    }
  }

  spinner.stop();
  ROS_INFO_STREAM_NAMED("hardware_interface", "Shutting down.");
  return 0;
}
