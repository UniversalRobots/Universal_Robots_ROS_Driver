#pragma once

#include "ur_modern_driver/bin_parser.h"
#include "ur_modern_driver/pipeline.h"
#include "ur_modern_driver/types.h"
#include <cstddef>
#include <inttypes.h>

class URRTPacketConsumer;

class RTPacket {
public:
    virtual bool parse_with(BinParser& bp) = 0;
    virtual bool consume_with(URRTPacketConsumer& consumer) = 0;
};

class RTShared {
protected:
    bool parse_shared1(BinParser& bp);
    bool parse_shared2(BinParser& bp);

public:
    double time;
    double q_target[6];
    double qd_target[6];
    double qdd_target[6];
    double i_target[6];
    double m_target[6];
    double q_actual[6];
    double qd_actual[6];
    double i_actual[6];

    //gap here depending on version

    double tcp_force[6];
    cartesian_coord_t tool_vector_actual;
    cartesian_coord_t tcp_speed_actual;

    //gap here depending on version

    uint64_t digital_input;
    double motor_temperatures[6];
    double controller_time;
    double robot_mode;

    static const size_t SIZE = sizeof(double) * 3
        + sizeof(double[6]) * 10
        + sizeof(cartesian_coord_t) * 2
        + sizeof(uint64_t);
};

class RTState_V1_6__7 : public RTShared, public RTPacket {
public:
    bool parse_with(BinParser& bp);
    virtual bool consume_with(URRTPacketConsumer& consumer);

    double3_t tool_accelerometer_values;

    static const size_t SIZE = RTShared::SIZE
        + sizeof(double3_t);

    static_assert(RTState_V1_6__7::SIZE == 632, "RTState_V1_6__7 has mismatched size!");
};

class RTState_V1_8 : public RTState_V1_6__7 {
public:
    bool parse_with(BinParser& bp);
    virtual bool consume_with(URRTPacketConsumer& consumer);

    double joint_modes[6];

    static const size_t SIZE = RTState_V1_6__7::SIZE
        + sizeof(double[6]);

    static_assert(RTState_V1_8::SIZE == 680, "RTState_V1_8 has mismatched size!");
};

class RTState_V3_0__1 : public RTShared, public RTPacket {
public:
    bool parse_with(BinParser& bp);
    virtual bool consume_with(URRTPacketConsumer& consumer);

    double i_control[6];
    cartesian_coord_t tool_vector_target;
    cartesian_coord_t tcp_speed_target;

    double joint_modes[6];
    double safety_mode;
    double3_t tool_accelerometer_values;
    double speed_scaling;
    double linear_momentum_norm;
    double v_main;
    double v_robot;
    double i_robot;
    double v_actual[6];

    static const size_t SIZE = RTShared::SIZE
        + sizeof(double[6]) * 3
        + sizeof(double3_t)
        + sizeof(cartesian_coord_t) * 2
        + sizeof(double) * 6;

    static_assert(RTState_V3_0__1::SIZE == 920, "RTState_V3_0__1 has mismatched size!");
};

class RTState_V3_2__3 : public RTState_V3_0__1 {
public:
    bool parse_with(BinParser& bp);
    virtual bool consume_with(URRTPacketConsumer& consumer);

    uint64_t digital_outputs;
    double program_state;

    static const size_t SIZE = RTState_V3_0__1::SIZE
        + sizeof(uint64_t)
        + sizeof(double);

    static_assert(RTState_V3_2__3::SIZE == 936, "RTState_V3_2__3 has mismatched size!");
};