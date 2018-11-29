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

class TrajectoryFollower : public ActionTrajectoryFollowerInterface
{
private:
  std::atomic<bool> running_;
  std::array<double, 6> last_positions_;
  URCommander &commander_;
  URServer server_;

  double servoj_time_, servoj_lookahead_time_, servoj_gain_;
  std::string program_;

  template <typename T>
  size_t append(uint8_t *buffer, T &val)
  {
    size_t s = sizeof(T);
    std::memcpy(buffer, &val, s);
    return s;
  }

  bool execute(std::array<double, 6> &positions, bool keep_alive);
  double interpolate(double t, double T, double p0_pos, double p1_pos, double p0_vel, double p1_vel);

public:
  TrajectoryFollower(URCommander &commander, std::string &reverse_ip, int reverse_port, bool version_3);

  bool start();
  bool execute(std::array<double, 6> &positions);
  bool execute(std::vector<TrajectoryPoint> &trajectory, std::atomic<bool> &interrupt);
  void stop();

  virtual ~TrajectoryFollower(){};
};
