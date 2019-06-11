// this is for emacs file handling -*- mode: c++; indent-tabs-mode: nil -*-

// -- BEGIN LICENSE BLOCK ----------------------------------------------
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
