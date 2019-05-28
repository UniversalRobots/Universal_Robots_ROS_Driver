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

#ifndef UR_RTDE_DRIVER_ROBOT_STATE_H_INCLUDED
#define UR_RTDE_DRIVER_ROBOT_STATE_H_INCLUDED

#include <sstream>

#include "ur_rtde_driver/primary/primary_package.h"
#include "ur_rtde_driver/primary/package_header.h"

namespace ur_driver
{
namespace primary_interface
{
enum class RobotStateType : uint8_t
{
  ROBOT_MODE_DATA = 0,
  JOINT_DATA = 1,
  TOOL_DATA = 2,
  MASTERBOARD_DATA = 3,
  CARTESIAN_INFO = 4,
  KINEMATICS_INFO = 5,
  CONFIGURATION_DATA = 6,
  FORCE_MODE_DATA = 7,
  ADDITIONAL_INFO = 8,
  CALIBRATION_DATA = 9
};

/*!
 * \brief Base class for a RobotState data packages will be used directly.
 */
class RobotState : public PrimaryPackage
{
public:
  RobotState() = delete;
  RobotState(const RobotStateType type) : state_type_(type)
  {
  }
  virtual ~RobotState() = default;

  virtual bool parseWith(comm::BinParser& bp)
  {
    return PrimaryPackage::parseWith(bp);
  }
  virtual std::string toString() const
  {
    std::stringstream ss;
    ss << "Type: " << static_cast<int>(state_type_) << std::endl;
    ss << PrimaryPackage::toString();
    return ss.str();
  }

private:
  RobotStateType state_type_;
};

}  // namespace primary_interface
}  // namespace ur_driver

#endif /* UR_RTDE_DRIVER_ROBOT_STATE_H_INCLUDED */
