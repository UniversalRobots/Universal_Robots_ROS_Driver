// this is for emacs file handling -*- mode: c++; indent-tabs-mode: nil -*-
//
// -- BEGIN LICENSE BLOCK ----------------------------------------------
// Copyright 2020 FZI Forschungszentrum Informatik
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
 * \date    2020-04-30
 *
 */
//----------------------------------------------------------------------

#ifndef UR_ROBOT_DRIVER_PRIMARY_PACKAGE_HANDLER_H_INCLUDED
#define UR_ROBOT_DRIVER_PRIMARY_PACKAGE_HANDLER_H_INCLUDED

namespace ur_driver
{
namespace primary_interface
{
/*!
 * \brief Interface for a class handling a primary interface package. Classes that implement this
 * interface with a specific package type will be able to handle packages of this type.
 */
template <typename PackageT>
class IPrimaryPackageHandler
{
public:
  IPrimaryPackageHandler() = default;
  virtual ~IPrimaryPackageHandler() = default;

  /*!
   * \brief Actual worker function
   *
   * \param pkg package that should be handled
   */
  virtual void handle(PackageT& pkg) = 0;

private:
  /* data */
};
}  // namespace primary_interface
}  // namespace ur_driver
#endif  // ifndef UR_ROBOT_DRIVER_PRIMARY_PACKAGE_HANDLER_H_INCLUDED
