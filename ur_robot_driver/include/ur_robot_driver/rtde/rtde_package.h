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

#ifndef UR_RTDE_DRIVER_RTDE_PACKAGE_H_INCLUDED
#define UR_RTDE_DRIVER_RTDE_PACKAGE_H_INCLUDED

#include "ur_robot_driver/rtde/package_header.h"
#include "ur_robot_driver/comm/package.h"

namespace ur_driver
{
namespace rtde_interface
{
/*!
 * brief The RTDEPackage is solely an abstraction level.
 * It inherits form the URPackage and is also a parent class for rtde_interface::TextMessage.
 */
class RTDEPackage : public comm::URPackage<PackageHeader>
{
public:
  /*!
   * \brief Creates a new RTDEPackage object.
   */
  RTDEPackage() = delete;
  RTDEPackage(const PackageType type) : type_(type)
  {
  }
  virtual ~RTDEPackage() = default;

  /*!
   * \brief Sets the attributes of the package by parsing a serialized representation of the
   * package.
   *
   * \param bp A parser containing a serialized version of the package
   *
   * \returns True, if the package was parsed successfully, false otherwise
   */
  virtual bool parseWith(comm::BinParser& bp);
  /*!
   * \brief Produces a human readable representation of the package object.
   *
   * \returns A string representing the object
   */
  virtual std::string toString() const;

protected:
  std::unique_ptr<uint8_t> buffer_;
  size_t buffer_length_;
  PackageType type_;
};

}  // namespace rtde_interface
}  // namespace ur_driver

#endif  // UR_RTDE_DRIVER_RTDE_PACKAGE_H_INCLUDED
