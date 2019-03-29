/*
 * Copyright 2017, 2018 Jarek Potiuk (low bandwidth trajectory follower)
 *
 * Copyright 2017, 2018 Simon Rasmussen (refactor)
 *
 * Copyright 2015, 2016 Thomas Timm Andersen (original version)
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
#include <vector>

struct TrajectoryPoint
{
  std::array<double, 6> positions;
  std::array<double, 6> velocities;
  std::chrono::microseconds time_from_start;

  TrajectoryPoint()
  {
  }

  TrajectoryPoint(std::array<double, 6> &pos, std::array<double, 6> &vel, std::chrono::microseconds tfs)
    : positions(pos), velocities(vel), time_from_start(tfs)
  {
  }
};

class ActionTrajectoryFollowerInterface
{
public:
  virtual bool start() = 0;
  virtual bool execute(std::vector<TrajectoryPoint> &trajectory, std::atomic<bool> &interrupt) = 0;
  virtual void stop() = 0;
  virtual ~ActionTrajectoryFollowerInterface(){};
};
