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
 * \date    2019-06-11
 *
 */
//----------------------------------------------------------------------

#ifndef UR_RTDE_DRIVER_UR_VERSION_INFORMATION_H_INCLUDED
#define UR_RTDE_DRIVER_UR_VERSION_INFORMATION_H_INCLUDED

#include <ur_robot_driver/types.h>

namespace ur_driver
{
/*!
 * \brief Struct containing a robot's version information
 */
struct VersionInformation
{
  VersionInformation()
  {
    major = 0;
    minor = 0;
    bugfix = 0;
    build = 0;
  }

  friend std::ostream& operator<<(std::ostream& os, const VersionInformation& version_info)
  {
    os << version_info.major << "." << version_info.minor << "." << version_info.bugfix << "-" << version_info.build;
    return os;
  }
  uint32_t major;   ///< Major version number
  uint32_t minor;   ///< Minor version number
  uint32_t bugfix;  ///< Bugfix version number
  uint32_t build;   ///< Build number
};
}  // namespace ur_driver

#endif  // ifndef UR_RTDE_DRIVER_UR_VERSION_INFORMATION_H_INCLUDED
