// this is for emacs file handling -*- mode: c++; indent-tabs-mode: nil -*-

// -- BEGIN LICENSE BLOCK ----------------------------------------------
// Copyright 2019 FZI Forschungszentrum Informatik
//
// Licensed under the Apache License, Text 2.0 (the "License");
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
 * \date    2020-04-23
 *
 */
//----------------------------------------------------------------------

#ifndef UR_RTDE_DRIVER_PRIMARY_TEXT_MESSAGE_H_INCLUDED
#define UR_RTDE_DRIVER_PRIMARY_TEXT_MESSAGE_H_INCLUDED

#include "ur_robot_driver/primary/robot_message.h"

namespace ur_driver
{
namespace primary_interface
{
/*!
 * \brief The TextMessage class handles the text messages sent via the primary UR interface.
 */
class TextMessage : public RobotMessage
{
public:
  TextMessage() = delete;
  /*!
   * \brief Creates a new TextMessage object to be filled from a package.
   *
   * \param timestamp Timestamp of the package
   * \param source The package's source
   */
  TextMessage(uint64_t timestamp, int8_t source)
    : RobotMessage(timestamp, source, RobotMessagePackageType::ROBOT_MESSAGE_TEXT)
  {
  }
  virtual ~TextMessage() = default;

  /*!
   * \brief Sets the attributes of the package by parsing a serialized representation of the
   * package.
   *
   * \param bp A parser containing a serialized text of the package
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

  std::string text_;
};
}  // namespace primary_interface
}  // namespace ur_driver

#endif  // ifndef UR_RTDE_DRIVER_PRIMARY_TEXT_MESSAGE_H_INCLUDED
