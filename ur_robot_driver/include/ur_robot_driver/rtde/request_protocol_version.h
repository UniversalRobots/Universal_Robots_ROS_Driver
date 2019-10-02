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
 * \author  Tristan Schnell schnell@fzi.de
 * \date    2019-04-09
 *
 */
//----------------------------------------------------------------------

#ifndef UR_RTDE_DRIVER_REQUEST_PROTOCOL_VERSION_H_INCLUDED
#define UR_RTDE_DRIVER_REQUEST_PROTOCOL_VERSION_H_INCLUDED

#include "ur_robot_driver/rtde/rtde_package.h"
#include "ur_robot_driver/rtde/package_header.h"

namespace ur_driver
{
namespace rtde_interface
{
/*!
 * \brief This class handles the robot's response after trying to set the used RTDE protocol version.
 */
class RequestProtocolVersion : public RTDEPackage
{
public:
  /*!
   * \brief Creates a new RequestProtocolVersion object.
   */
  RequestProtocolVersion() : RTDEPackage(PackageType::RTDE_REQUEST_PROTOCOL_VERSION)
  {
  }
  virtual ~RequestProtocolVersion() = default;

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

  uint8_t accepted_;
};

/*!
 * \brief This class handles producing a request towards the robot to use a specific RTDE protocol
 * version.
 */
class RequestProtocolVersionRequest : public RTDEPackage
{
public:
  /*!
   * \brief Creates a new RequestProtocolVersionRequest object.
   */
  RequestProtocolVersionRequest() : RTDEPackage(PackageType::RTDE_REQUEST_PROTOCOL_VERSION)
  {
  }

  virtual ~RequestProtocolVersionRequest() = default;

  /*!
   * \brief Generates a serialized package.
   *
   * \param buffer Buffer to fill with the serialization
   * \param version RTDE protocol version to request usage of
   *
   * \returns The total size of the serialized package
   */
  static size_t generateSerializedRequest(uint8_t* buffer, uint16_t version);

  uint16_t protocol_version_;

private:
  static const uint16_t PAYLOAD_SIZE = sizeof(uint16_t);
};

}  // namespace rtde_interface
}  // namespace ur_driver

#endif  // UR_RTDE_DRIVER_REQUEST_PROTOCOL_VERSION_H_INCLUDED
