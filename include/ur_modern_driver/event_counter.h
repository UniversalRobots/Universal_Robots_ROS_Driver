#pragma once

#include <chrono>
#include <cstdlib>
#include "ur_modern_driver/log.h"
#include "ur_modern_driver/ur/consumer.h"

class EventCounter : public URRTPacketConsumer
{
private:
  typedef std::chrono::high_resolution_clock Clock;
  Clock::time_point events_[250];
  size_t idx_ = 0;

  Clock::time_point last_;

public:
  void trigger()
  {
    // auto now = Clock::now();
    // LOG_INFO("Time diff: %d ms", std::chrono::duration_cast<std::chrono::microseconds>(now - last_));
    // last_ = now;
    // return;

    events_[idx_] = Clock::now();
    idx_ += 1;

    if (idx_ > 250)
    {
      std::chrono::time_point<std::chrono::high_resolution_clock> t_min =
          std::chrono::time_point<std::chrono::high_resolution_clock>::max();
      std::chrono::time_point<std::chrono::high_resolution_clock> t_max =
          std::chrono::time_point<std::chrono::high_resolution_clock>::min();

      for (auto const& e : events_)
      {
        if (e < t_min)
          t_min = e;
        if (e > t_max)
          t_max = e;
      }

      auto diff = t_max - t_min;
      auto secs = std::chrono::duration_cast<std::chrono::seconds>(diff).count();
      auto ms = std::chrono::duration_cast<std::chrono::microseconds>(diff).count();
      std::chrono::duration<double> test(t_max - t_min);
      LOG_INFO("Recieved 250 messages at %f Hz", (250.0 / test.count()));
      idx_ = 0;
    }
  }

public:
  bool consume(RTState_V1_6__7& state)
  {
    trigger();
    return true;
  }
  bool consume(RTState_V1_8& state)
  {
    trigger();
    return true;
  }
  bool consume(RTState_V3_0__1& state)
  {
    trigger();
    return true;
  }
  bool consume(RTState_V3_2__3& state)
  {
    trigger();
    return true;
  }

  void setupConsumer()
  {
    last_ = Clock::now();
  }
  void teardownConsumer()
  {
  }
  void stopConsumer()
  {
  }
};