#pragma once

#include <array>
#include <atomic>
#include <cstddef>
#include <cstring>
#include <string>
#include <inttypes.h>
#include "ur_modern_driver/ur/commander.h"
#include "ur_modern_driver/ur/server.h"
#include "ur_modern_driver/ur/stream.h"

class TrajectoryFollower 
{
private:
  const int32_t MULT_JOINTSTATE_ = 1000000;
  double servoj_time_, servoj_lookahead_time_, servoj_gain_;
  std::atomic<bool> running_;
  std::array<double, 6> last_positions_;
  URCommander &commander_;
  URServer server_;
  URStream stream_;
  std::string program_;

  template <typename T>
  size_t append(uint8_t *buffer, T &val)
  {
    size_t s = sizeof(T);
    std::memcpy(buffer, &val, s);
    return s;
  }

  bool execute(std::array<double, 6> &positions, bool keep_alive);  

public:
  TrajectoryFollower(URCommander &commander, int reverse_port, bool version_3);

  std::string buildProgram(bool version_3);

  bool start();
  bool execute(std::array<double, 6> &positions);
  void stop();
  void halt(); //maybe
};