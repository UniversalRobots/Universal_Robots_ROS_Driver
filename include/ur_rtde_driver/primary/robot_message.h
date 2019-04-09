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
 * \author  Lea Steffen steffen@fzi.de
 * \date    2019-04-01
 *
 */
//----------------------------------------------------------------------

#ifndef UR_RTDE_DRIVER_ROBOT_MESSAGE_H_INCLUDED
#define UR_RTDE_DRIVER_ROBOT_MESSAGE_H_INCLUDED

#include "ur_rtde_driver/primary/primary_package.h"

namespace ur_driver
{
namespace primary_interface
{
class RobotMessage : public PrimaryPackage
{
public:
  RobotMessage(const uint64_t timestamp, const uint8_t source) : timestamp_(timestamp), source_(source)
  {
  }
  virtual ~RobotMessage() = default;

  virtual bool parseWith(comm::BinParser& bp);
  virtual std::string toString() const;

  uint64_t timestamp_;
  uint8_t source_;
  uint8_t message_type_;
};

}  // namespace primary_interface
}  // namespace ur_driver

#endif /* UR_RTDE_DRIVER_ROBOT_MESSAGE_H_INCLUDED */
