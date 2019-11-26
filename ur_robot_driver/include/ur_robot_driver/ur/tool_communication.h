
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
 * \author  Felix Exner exner@fzi.de
 * \date    2019-06-06
 *
 */
//----------------------------------------------------------------------

#ifndef UR_RTDE_DRIVER_UR_TOOL_COMMUNICATION_H_INCLUDED
#define UR_RTDE_DRIVER_UR_TOOL_COMMUNICATION_H_INCLUDED

#include "ur_robot_driver/types.h"
#include <set>

namespace ur_driver
{
/*!
 * \brief Possible values for the tool voltage
 */
enum class ToolVoltage : int
{
  OFF = 0,    ///< 0V
  _12V = 12,  ///< 12V
  _24V = 24   ///< 24V
};

/*!
 * \brief Possible values for th parity flag
 */
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

  /*!
   * \brief Create a new Limited object
   *
   * \param lower Lower boundary used for this Limited object
   * \param upper Upper boundary used for this Limited object
   */
  Limited(const T lower, const T upper) : lower_(lower), upper_(upper)
  {
    data_ = lower_;
  }

  /*!
   * \brief Set the data field with a given value
   *
   * If the given value is out of the configured range, an exception is thrown.
   *
   * \param data
   */
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

  /*!
   * \brief Returns the data stored in this object
   */
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

  /*!
   * \brief Setup the tool voltage that will be configured on the robot. This will not immediately
   * change values on the robot, it will only be stored inside the ToolCommSetup object.
   */
  void setToolVoltage(const ToolVoltage tool_voltage)
  {
    tool_voltage_ = tool_voltage;
  }

  /*!
   * \brief Return the tool voltage currently stored
   */
  ToolVoltage getToolVoltage() const
  {
    return tool_voltage_;
  }

  /*!
   * \brief Setup the tool communication parity that will be configured on the robot. This will not immediately
   * change values on the robot, it will only be stored inside the ToolCommSetup object.
   */
  void setParity(const Parity parity)
  {
    parity_ = parity;
  }
  /*!
   * \brief Return the parity currently stored
   */
  Parity getParity() const
  {
    return parity_;
  }

  /*!
   * \brief Setup the tool communication baud rate that will be configured on the robot. This will not immediately
   * change values on the robot, it will only be stored inside the ToolCommSetup object.
   *
   * \param baud_rate must be one of baud_rates_allowed_ or an exception will be thrown
   */
  void setBaudRate(const uint32_t baud_rate);
  /*!
   * \brief Return the baud rate currently stored
   */
  uint32_t getBaudRate() const
  {
    return baud_rate_;
  };

  /*!
   * \brief Setup the tool communication number of stop bits that will be configured on the robot. This will not
   * immediately change values on the robot, it will only be stored inside the ToolCommSetup object.
   *
   * \param stop_bits must be inside [1,2] or this will throw an exception.
   */
  void setStopBits(const StopBitsT::Datatype stop_bits)
  {
    stop_bits_.setData(stop_bits);
  }
  /*!
   * \brief Return the number of stop bits currently stored
   */
  StopBitsT::Datatype getStopBits() const
  {
    return stop_bits_.getData();
  }

  /*!
   * \brief Setup the tool communication number of idle chars for the rx channel that will be configured on the robot.
   * This will not immediately change values on the robot, it will only be stored inside the ToolCommSetup object.
   *
   * \param rx_idle_chars must be inside [1.0, 40] or this will throw an exception.
   */
  void setRxIdleChars(const RxIdleCharsT::Datatype rx_idle_chars)
  {
    rx_idle_chars_.setData(rx_idle_chars);
  }
  /*!
   * \brief Return the number of rx idle chars currently stored
   */
  RxIdleCharsT::Datatype getRxIdleChars() const
  {
    return rx_idle_chars_.getData();
  }

  /*!
   * \brief Setup the tool communication number of idle chars for the tx channel that will be configured on the robot.
   * This will not immediately change values on the robot, it will only be stored inside the ToolCommSetup object.
   *
   * \param tx_idle_chars must be inside [0.0, 40] or this will throw an exception.
   */
  void setTxIdleChars(const TxIdleCharsT::Datatype tx_idle_chars)
  {
    tx_idle_chars_.setData(tx_idle_chars);
  }
  /*!
   * \brief Return the number of tx idle chars currently stored
   */
  TxIdleCharsT::Datatype getTxIdleChars() const
  {
    return tx_idle_chars_.getData();
  }

private:
  const std::set<uint32_t> baud_rates_allowed_{ 9600,
                                                19200,
                                                38400,
                                                57600,
                                                115200,
                                                static_cast<uint32_t>(1e6),
                                                static_cast<uint32_t>(2e6),
                                                static_cast<uint32_t>(5e6) };

  ToolVoltage tool_voltage_;
  Parity parity_;
  uint32_t baud_rate_;
  StopBitsT stop_bits_;
  RxIdleCharsT rx_idle_chars_;
  TxIdleCharsT tx_idle_chars_;
};
}  // namespace ur_driver
#endif  // ifndef UR_RTDE_DRIVER_UR_TOOL_COMMUNICATION_H_INCLUDED
