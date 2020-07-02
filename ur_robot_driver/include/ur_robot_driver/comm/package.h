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
 * \date    2019-04-08
 *
 */
//----------------------------------------------------------------------
#ifndef UR_RTDE_DRIVER_PACKAGE_H_INCLUDED
#define UR_RTDE_DRIVER_PACKAGE_H_INCLUDED

#include "ur_robot_driver/comm/bin_parser.h"

namespace ur_driver
{
namespace comm
{
/*!
 * \brief The URPackage a parent class. From that two implementations are inherited,
 * one for the primary, one for the rtde interface (primary_interface::primaryPackage;
 * rtde_interface::rtdePackage). The URPackage makes use of the template HeaderT.
 */
template <typename HeaderT>
class URPackage
{
public:
  /*!
   * \brief Creates a new URPackage object.
   */

  URPackage() = default;
  virtual ~URPackage() = default;

  /*!
   * \brief Sets the attributes of the package by parsing a serialized representation of the
   * package.
   *
   * \param bp A parser containing a serialized version of the package
   *
   * \returns True, if the package was parsed successfully, false otherwise
   */
  virtual bool parseWith(BinParser& bp) = 0;

  /*!
   * \brief Produces a human readable representation of the package object.
   *
   * \returns A string representing the object
   */
  virtual std::string toString() const = 0;

  using HeaderType = HeaderT;

private:
  HeaderT header_;
};
}  // namespace comm
}  // namespace ur_driver
#endif  // ifndef UR_RTDE_DRIVER_PACKAGE_H_INCLUDED
