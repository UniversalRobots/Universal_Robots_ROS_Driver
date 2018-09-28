#pragma once

#include <inttypes.h>
#include <cstddef>
#include "ur_modern_driver/bin_parser.h"
#include "ur_modern_driver/pipeline.h"
#include "ur_modern_driver/types.h"

class URRTPacketConsumer;

class RTPacket
{
public:
  virtual bool parseWith(BinParser& bp) = 0;
  virtual bool consumeWith(URRTPacketConsumer& consumer) = 0;
};

class RTShared
{
protected:
  void parse_shared1(BinParser& bp);
  void parse_shared2(BinParser& bp);

public:
  double time;
  std::array<double, 6> q_target;
  std::array<double, 6> qd_target;
  std::array<double, 6> qdd_target;
  std::array<double, 6> i_target;
  std::array<double, 6> m_target;
  std::array<double, 6> q_actual;
  std::array<double, 6> qd_actual;
  std::array<double, 6> i_actual;

  // gap here depending on version

  std::array<double, 6> tcp_force;

  // does not contain "_actual" postfix in V1_X but
  // they're the same fields so share anyway
  cartesian_coord_t tool_vector_actual;
  cartesian_coord_t tcp_speed_actual;

  // gap here depending on version

  uint64_t digital_inputs;
  std::array<double, 6> motor_temperatures;
  double controller_time;
  double robot_mode;

  static const size_t SIZE =
      sizeof(double) * 3 + sizeof(double[6]) * 10 + sizeof(cartesian_coord_t) * 2 + sizeof(uint64_t);
};

class RTState_V1_6__7 : public RTShared, public RTPacket
{
public:
  bool parseWith(BinParser& bp);
  virtual bool consumeWith(URRTPacketConsumer& consumer);

  double3_t tool_accelerometer_values;

  static const size_t SIZE = RTShared::SIZE + sizeof(double3_t);

  static_assert(RTState_V1_6__7::SIZE == 632, "RTState_V1_6__7 has mismatched size!");
};

class RTState_V1_8 : public RTState_V1_6__7
{
public:
  bool parseWith(BinParser& bp);
  virtual bool consumeWith(URRTPacketConsumer& consumer);

  std::array<double, 6> joint_modes;

  static const size_t SIZE = RTState_V1_6__7::SIZE + sizeof(double[6]);

  static_assert(RTState_V1_8::SIZE == 680, "RTState_V1_8 has mismatched size!");
};

class RTState_V3_0__1 : public RTShared, public RTPacket
{
public:
  bool parseWith(BinParser& bp);
  virtual bool consumeWith(URRTPacketConsumer& consumer);

  std::array<double, 6> i_control;
  cartesian_coord_t tool_vector_target;
  cartesian_coord_t tcp_speed_target;

  std::array<double, 6> joint_modes;
  double safety_mode;
  double3_t tool_accelerometer_values;
  double speed_scaling;
  double linear_momentum_norm;
  double v_main;
  double v_robot;
  double i_robot;
  std::array<double, 6> v_actual;

  static const size_t SIZE =
      RTShared::SIZE + sizeof(double[6]) * 3 + sizeof(double3_t) + sizeof(cartesian_coord_t) * 2 + sizeof(double) * 6;

  static_assert(RTState_V3_0__1::SIZE == 920, "RTState_V3_0__1 has mismatched size!");
};

class RTState_V3_2__3 : public RTState_V3_0__1
{
public:
  bool parseWith(BinParser& bp);
  virtual bool consumeWith(URRTPacketConsumer& consumer);

  uint64_t digital_outputs;
  double program_state;

  static const size_t SIZE = RTState_V3_0__1::SIZE + sizeof(uint64_t) + sizeof(double);

  static_assert(RTState_V3_2__3::SIZE == 936, "RTState_V3_2__3 has mismatched size!");
};