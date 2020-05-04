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
 * \date    2020-04-30
 *
 */
//----------------------------------------------------------------------
#ifndef UR_ROBOT_DRIVER_PRIMARY_CLIENT_H_INCLUDED
#define UR_ROBOT_DRIVER_PRIMARY_CLIENT_H_INCLUDED

#include <ur_robot_driver/primary/primary_parser.h>
#include <ur_robot_driver/comm/producer.h>
#include <ur_robot_driver/comm/stream.h>
#include <ur_robot_driver/comm/pipeline.h>
#include <ur_robot_driver/ur/calibration_checker.h>
#include <ur_robot_driver/primary/primary_consumer.h>

namespace ur_driver
{
namespace primary_interface
{
class PrimaryClient
{
public:
  PrimaryClient() = delete;
  PrimaryClient(const std::string& robot_ip, const std::string& calibration_checksum);
  virtual ~PrimaryClient() = default;

  /*!
   * \brief Sends a custom script program to the robot.
   *
   * The given code must be valid according the UR Scripting Manual.
   *
   * \param script_code URScript code that shall be executed by the robot.
   *
   * \returns true on successful upload, false otherwise.
   */
  bool sendScript(const std::string& script_code);

  /*!
   * \brief Checks if the kinematics information in the used model fits the actual robot.
   *
   * \param checksum Hash of the used kinematics information
   */
  void checkCalibration(const std::string& checksum);

private:
  std::string robot_ip_;
  PrimaryParser parser_;
  std::unique_ptr<PrimaryConsumer> consumer_;
  comm::INotifier notifier_;
  std::unique_ptr<comm::URProducer<PrimaryPackage>> producer_;
  std::unique_ptr<comm::URStream<PrimaryPackage>> stream_;
  std::unique_ptr<comm::Pipeline<PrimaryPackage>> pipeline_;
};

}  // namespace primary_interface
}  // namespace ur_driver

#endif  // ifndef UR_ROBOT_DRIVER_PRIMARY_CLIENT_H_INCLUDED
