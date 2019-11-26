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

#include "ur_robot_driver/ur/tool_communication.h"

namespace ur_driver
{
ToolCommSetup::ToolCommSetup()
  : tool_voltage_(ToolVoltage::OFF)
  , parity_(Parity::ODD)
  , baud_rate_(9600)
  , stop_bits_(1, 2)
  , rx_idle_chars_(1.0, 40.0)
  , tx_idle_chars_(0.0, 40.0)
{
}

void ToolCommSetup::setBaudRate(const uint32_t baud_rate)
{
  if (baud_rates_allowed_.find(baud_rate) != baud_rates_allowed_.end())
  {
    baud_rate_ = baud_rate;
  }
  else
  {
    throw std::runtime_error("Provided baud rate is not allowed");
  }
}
}  // namespace ur_driver
