/*
 * Copyright 2017, 2018 Jarek Potiuk (low bandwidth trajectory follower)
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

#pragma once

#include <inttypes.h>
#include <array>
#include <atomic>
#include <cstddef>
#include <cstring>
#include <string>
#include <thread>
#include <vector>
#include "ur_modern_driver/log.h"
#include "ur_modern_driver/ros/action_trajectory_follower_interface.h"
#include "ur_modern_driver/ur/commander.h"
#include "ur_modern_driver/ur/server.h"

class LowBandwidthTrajectoryFollower : public ActionTrajectoryFollowerInterface
{
private:
  std::atomic<bool> running_;
  std::array<double, 6> last_positions_;
  URCommander &commander_;
  URServer server_;

  double time_interval_, servoj_time_, servoj_time_waiting_, max_waiting_time_, servoj_gain_, servoj_lookahead_time_,
      max_joint_difference_;

  std::string program_;

  bool execute(const std::array<double, 6> &positions, const std::array<double, 6> &velocities, double sample_number,
               double time_in_seconds);

public:
  LowBandwidthTrajectoryFollower(URCommander &commander, std::string &reverse_ip, int reverse_port, bool version_3);

  bool start();
  bool execute(std::vector<TrajectoryPoint> &trajectory, std::atomic<bool> &interrupt);
  void stop();

  virtual ~LowBandwidthTrajectoryFollower(){};
};
