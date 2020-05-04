/*
 * Copyright 2019, FZI Forschungszentrum Informatik (templating)
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
#include <chrono>
#include "ur_robot_driver/comm/pipeline.h"
#include "ur_robot_driver/comm/parser.h"
#include "ur_robot_driver/comm/stream.h"
#include "ur_robot_driver/comm/package.h"
#include "ur_robot_driver/exceptions.h"

namespace ur_driver
{
namespace comm
{
/*!
 * \brief A general producer for URPackages. Implements funcionality to produce packages by
 * reading and parsing from a byte stream.
 *
 * @tparam HeaderT Header type of packages to produce.
 */
template <typename T>
class URProducer : public IProducer<T>
{
private:
  URStream<T>& stream_;
  Parser<T>& parser_;
  std::chrono::seconds timeout_;

  bool running_;

public:
  /*!
   * \brief Creates a URProducer object, registering a stream and a parser.
   *
   * \param stream The stream to read from
   * \param parser The parser to use to interpret received byte information
   */
  URProducer(URStream<T>& stream, Parser<T>& parser) : stream_(stream), parser_(parser), timeout_(1), running_(false)
  {
  }

  /*!
   * \brief Triggers the stream to connect to the robot.
   */
  void setupProducer() override
  {
    timeval tv;
    tv.tv_sec = 1;
    tv.tv_usec = 0;
    stream_.setReceiveTimeout(tv);
    if (!stream_.connect())
    {
      throw UrException("Failed to connect to robot. Please check if the robot is booted and connected.");
    }
  }
  /*!
   * \brief Tears down the producer. Currently no special handling needed.
   */
  void teardownProducer() override
  {
    stopProducer();
  }
  /*!
   * \brief Stops the producer. Currently no functionality needed.
   */
  void stopProducer() override
  {
    running_ = false;
  }

  void startProducer() override
  {
    running_ = true;
  }

  /*!
   * \brief Attempts to read byte stream from the robot and parse it as a URPackage.
   *
   * \param products Unique pointer to hold the produced package
   *
   * \returns Success of reading and parsing the package
   */
  bool tryGet(std::vector<std::unique_ptr<T>>& products) override
  {
    // TODO This function has become really ugly! That should be refactored!

    // 4KB should be enough to hold any packet received from UR
    uint8_t buf[4096];
    size_t read = 0;
    // expoential backoff reconnects
    while (true)
    {
      if (stream_.read(buf, sizeof(buf), read))
      {
        // reset sleep amount
        timeout_ = std::chrono::seconds(1);
        BinParser bp(buf, read);
        return parser_.parse(bp, products);
      }

      if (!running_)
        return true;

      if (stream_.closed())
        return false;

      LOG_WARN("Failed to read from stream, reconnecting in %ld seconds...", timeout_.count());
      std::this_thread::sleep_for(timeout_);

      if (stream_.connect())
        continue;

      auto next = timeout_ * 2;
      if (next <= std::chrono::seconds(120))
        timeout_ = next;
    }

    return false;
  }
};
}  // namespace comm
}  // namespace ur_driver
