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

#ifndef UR_RTDE_DRIVER_VERSION_MESSAGE_H_INCLUDED
#define UR_RTDE_DRIVER_VERSION_MESSAGE_H_INCLUDED

#include "ur_robot_driver/primary/robot_message.h"

namespace ur_driver
{
namespace primary_interface
{
/*!
 * \brief The VersionMessage class handles the version messages sent via the primary UR interface.
 */
class VersionMessage : public RobotMessage
{
public:
  VersionMessage() = delete;
  /*!
   * \brief Creates a new VersionMessage object to be filled from a package.
   *
   * \param timestamp Timestamp of the package
   * \param source The package's source
   */
  VersionMessage(uint64_t timestamp, uint8_t source) : RobotMessage(timestamp, source)
  {
  }
  virtual ~VersionMessage() = default;

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
   * \brief Consume this package with a specific consumer.
   *
   * \param consumer Placeholder for the consumer calling this
   *
   * \returns true on success
   */
  virtual bool consumeWith(AbstractPrimaryConsumer& consumer);

  /*!
   * \brief Produces a human readable representation of the package object.
   *
   * \returns A string representing the object
   */
  virtual std::string toString() const;

  int8_t project_name_length_;
  std::string project_name_;
  uint8_t major_version_;
  uint8_t minor_version_;
  int32_t svn_version_;
  int32_t build_number_;
  std::string build_date_;
};
}  // namespace primary_interface
}  // namespace ur_driver

#endif  // ifndef UR_RTDE_DRIVER_VERSION_MESSAGE_H_INCLUDED
