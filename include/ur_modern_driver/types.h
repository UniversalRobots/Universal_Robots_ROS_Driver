#pragma once

#include <inttypes.h>

typedef struct {
    double x, y, z;
} double3_t;

typedef struct {
    double3_t position;
    double3_t rotation;
} cartesian_coord_t;