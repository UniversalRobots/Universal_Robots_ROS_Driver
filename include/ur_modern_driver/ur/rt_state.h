#pragma once

#include <cstddef>
#include <inttypes.h>
#include "ur_modern_driver/types.h"
#include "ur_modern_driver/bin_parser.h"

class RTState_V1_6__7 {
public:
    bool parse_with(BinParser &bp);

    double time;
    double q_target[6];
    double qd_target[6];
    double qdd_target[6];
    double i_target[6];
    double m_target[6];
    double q_actual[6];
    double qd_actual[6];
    double i_actual[6];
    double3_t tool_accelerometer_values;
    double tcp_force[6];
    cartesian_coord_t tool_vector;
    double tcp_speed[6];
    uint64_t digital_input;
    double motor_temperatures[6];
    double controller_time;
    double robot_mode;

    static const size_t SIZE = sizeof(double) * 3 
        + sizeof(double[6]) * 11
        + sizeof(double3_t)
        + sizeof(cartesian_coord_t)
        + sizeof(uint64_t);


    static_assert(RTState_V1_6__7::SIZE == 632, "RTState_V1_6__7 has mismatched size!");
};

class RTState_V1_8 : public RTState_V1_6__7 {
public:
    bool parse_with(BinParser &bp);

    double joint_modes[6];

    static const size_t SIZE = RTState_V1_6__7::SIZE
        + sizeof(double[6]);

    static_assert(RTState_V1_8::SIZE == 680, "RTState_V1_8 has mismatched size!");
};

class RTState_V3_0__1 {
public:
    //bool parse_with(BinParser &bp);

    double m_actual[6];
    double i_control[6];
};