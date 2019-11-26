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
 * \author  Lea Steffen steffen@fzi.de
 * \date    2019-04-01
 *
 */
//----------------------------------------------------------------------

#ifndef UR_RTDE_DRIVER_PRIMARY_INTERFACE_H_INCLUDED
#define UR_RTDE_DRIVER_PRIMARY_INTERFACE_H_INCLUDED

#include <inttypes.h>
#include <cstddef>
#include <endian.h>
#include "ur_robot_driver/types.h"

namespace ur_driver
{
namespace primary_interface
{
static const int UR_PRIMARY_PORT = 30001;
static const int UR_SECONDARY_PORT = 30002;
/*!
 * \brief Possible RobotPackage types
 */
enum class RobotPackageType : int8_t
{
  DISCONNECT = -1,
  ROBOT_STATE = 16,
  ROBOT_MESSAGE = 20,
  HMC_MESSAGE = 22,
  MODBUS_INFO_MESSAGE = 5,
  SAFETY_SETUP_BROADCAST_MESSAGE = 23,
  SAFETY_COMPLIANCE_TOLERANCES_MESSAGE = 24,
  PROGRAM_STATE_MESSAGE = 25
};

/*!
 * \brief This class represents the header for primary packages.
 */
class PackageHeader
{
public:
  PackageHeader() = default;
  virtual ~PackageHeader() = default;

  using _package_size_type = int32_t;

  /*!
   * \brief Reads a buffer, interpreting the next bytes as the size of the contained package.
   *
   * \param buf The given byte stream containing a serialized package
   *
   * \returns The size of the given serialized package
   */
  static size_t getPackageLength(uint8_t* buf)
  {
    return be32toh(*(reinterpret_cast<_package_size_type*>(buf)));
  }
};
}  // namespace primary_interface
}  // namespace ur_driver

#endif  // ifndef UR_RTDE_DRIVER_PRIMARY_INTERFACE_H_INCLUDED
