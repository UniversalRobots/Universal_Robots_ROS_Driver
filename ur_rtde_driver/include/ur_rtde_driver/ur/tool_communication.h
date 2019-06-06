
// this is for emacs file handling -*- mode: c++; indent-tabs-mode: nil -*-

// -- BEGIN LICENSE BLOCK ----------------------------------------------
// Copyright 2019 FZI Forschungszentrum Informatik
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
// -- END LICENSE BLOCK ------------------------------------------------

//----------------------------------------------------------------------
/*!\file
 *
 * \author  Felix Mauch mauch@fzi.de
 * \date    2019-06-06
 *
 */
//----------------------------------------------------------------------

#ifndef UR_RTDE_DRIVER_UR_TOOL_COMMUNICATION_H_INCLUDED
#define UR_RTDE_DRIVER_UR_TOOL_COMMUNICATION_H_INCLUDED

#include "ur_rtde_driver/types.h"
#include <set>

namespace ur_driver
{
enum class ToolVoltage : int
{
  OFF = 0,
  _12V = 12,
  _24V = 24
};

enum class Parity : int
{
  NONE = 0,
  ODD = 1,
  EVEN = 2
};

/*!
 * \brief Helper class that represents a numeric value with a lower and an upper boundary.
 *
 * @tparam T any type for which a comparison exists.
 */
template <class T>
class Limited
{
public:
  Limited() = delete;
  ~Limited() = default;

  using Datatype = T;

  Limited(const T lower, const T upper) : lower_(lower), upper_(upper)
  {
    data_ = lower_;
  }

  void setData(const T data)
  {
    if (data >= lower_ && data <= upper_)
    {
      data_ = data;
    }
    else
    {
      throw std::runtime_error("Given data is out of range");
    }
  }

  T getData() const
  {
    return data_;
  }

private:
  T data_;
  const T lower_;
  const T upper_;
};

/*!
 * \brief Class holding a tool communication configuration
 */
class ToolCommSetup
{
public:
  ToolCommSetup();
  ~ToolCommSetup() = default;

  using StopBitsT = Limited<uint32_t>;
  using RxIdleCharsT = Limited<float>;
  using TxIdleCharsT = Limited<float>;

  void setToolVoltage(const ToolVoltage tool_voltage)
  {
    tool_voltage_ = tool_voltage;
  }
  ToolVoltage getToolVoltage() const
  {
    return tool_voltage_;
  }

  void setParity(const Parity parity)
  {
    parity_ = parity;
  }
  Parity getParity() const
  {
    return parity_;
  }

  void setBaudRate(const uint32_t baud_rate);
  uint32_t getBaudRate() const
  {
    return baud_rate_;
  };

  void setStopBits(const StopBitsT::Datatype stop_bits)
  {
    stop_bits_.setData(stop_bits);
  }
  StopBitsT::Datatype getStopBits() const
  {
    return stop_bits_.getData();
  }

  void setRxIdleChars(const RxIdleCharsT::Datatype rx_idle_chars)
  {
    rx_idle_chars_.setData(rx_idle_chars);
  }
  RxIdleCharsT::Datatype getRxIdleChars() const
  {
    return rx_idle_chars_.getData();
  }

  void setTxIdleChars(const TxIdleCharsT::Datatype tx_idle_chars)
  {
    tx_idle_chars_.setData(tx_idle_chars);
  }
  TxIdleCharsT::Datatype getTxIdleChars() const
  {
    return tx_idle_chars_.getData();
  }

private:
  const std::set<uint32_t> baud_rates_{ 9600, 19200, 38400, 57600, 115200, 10000000, 2000000, 5000000 };

  ToolVoltage tool_voltage_;
  Parity parity_;
  uint32_t baud_rate_;
  StopBitsT stop_bits_;
  RxIdleCharsT rx_idle_chars_;
  TxIdleCharsT tx_idle_chars_;
};
}  // namespace ur_driver
#endif  // ifndef UR_RTDE_DRIVER_UR_TOOL_COMMUNICATION_H_INCLUDED
