
// this is for emacs file handling -*- mode: c++; indent-tabs-mode: nil -*-

// -- BEGIN LICENSE BLOCK ----------------------------------------------
// -- END LICENSE BLOCK ------------------------------------------------

//----------------------------------------------------------------------
/*!\file
 *
 * \author  Lea Steffen steffen@fzi.de
 * \date    2019-04-01
 *
 */
//----------------------------------------------------------------------

#ifndef UR_RTDE_DRIVER_RTDE__HEADER_H_INCLUDED
#define UR_RTDE_DRIVER_RTDE__HEADER_H_INCLUDED

#include <cstddef>
#include <endian.h>
#include "ur_rtde_driver/types.h"

namespace ur_driver
{
namespace rtde_interface
{
enum class PackageType
{
  RTDE_REQUEST_PROTOCOL_VERSION = 86,       // ascii V
  RTDE_GET_URCONTROL_VERSION = 118,         // ascii v
  RTDE_TEXT_MESSAGE = 77,                   // ascii M
  RTDE_DATA_PACKAGE = 85,                   // ascii U
  RTDE_CONTROL_PACKAGE_SETUP_OUTPUTS = 79,  // ascii O
  RTDE_CONTROL_PACKAGE_SETUP_INPUTS = 73,   // ascii I
  RTDE_CONTROL_PACKAGE_START = 83,          // ascii S
  RTDE_CONTROL_PACKAGE_PAUSE = 80           // ascii P
};

class PackageHeader
{
public:
  PackageHeader() = default;
  virtual ~PackageHeader() = default;
  using _package_size_type = uint16_t;

  PackageHeader(PackageType& type) : package_type_(type){};
  PackageHeader(_package_size_type& size, PackageType& type) : package_size_(size), package_type_(type){};

  static size_t getPackageLength(uint8_t* buf)
  {
    return be16toh(*(reinterpret_cast<_package_size_type*>(buf)));
  }

private:
  _package_size_type package_size_;
  PackageType package_type_;
};

}  // namespace rtde_interface
}  // namespace ur_driver

#endif  // #ifndef UR_RTDE_DRIVER_RTDE__HEADER_H_INCLUDED
