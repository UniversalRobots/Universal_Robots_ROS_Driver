#pragma once

#include <inttypes.h>

struct double3_t
{
  double x, y, z;
};

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