/*
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

namespace ur_driver
{
// TODO:Replace
struct double3_t
{
  double x, y, z;
};

using vector3d_t = std::array<double, 3>;
using vector6d_t = std::array<double, 6>;
using vector6int32_t = std::array<int32_t, 6>;
using vector6uint32_t = std::array<uint32_t, 6>;

struct cartesian_coord_t
{
  double3_t position;
  double3_t rotation;
};

inline bool operator==(const double3_t& lhs, const double3_t& rhs)
{
  return lhs.x == rhs.x && lhs.y == rhs.y && lhs.z == rhs.z;
}

inline bool operator==(const cartesian_coord_t& lhs, const cartesian_coord_t& rhs)
{
  return lhs.position == rhs.position && lhs.rotation == rhs.rotation;
}
}  // namespace ur_driver
