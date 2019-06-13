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
 * \date    2019-06-11
 *
 */
//----------------------------------------------------------------------

#ifndef UR_RTDE_DRIVER_UR_VERSION_INFORMATION_H_INCLUDED
#define UR_RTDE_DRIVER_UR_VERSION_INFORMATION_H_INCLUDED

namespace ur_driver
{
struct VersionInformation
{
  VersionInformation()
  {
    major = 0;
    minor = 0;
    bugfix = 0;
    build = 0;
  }
  uint32_t major;
  uint32_t minor;
  uint32_t bugfix;
  uint32_t build;
};
}  // namespace ur_driver

#endif  // ifndef UR_RTDE_DRIVER_UR_VERSION_INFORMATION_H_INCLUDED
