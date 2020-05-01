// this is for emacs file handling -*- mode: c++; indent-tabs-mode: nil -*-

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
//
// -- END LICENSE BLOCK ------------------------------------------------

//----------------------------------------------------------------------
/*!\file
 *
 * \author  Felix Exner exner@fzi.de
 * \date    2020-04-30
 *
 */
//----------------------------------------------------------------------

#ifndef UR_ROBOT_DRIVER_PRIMARY_SHELL_CONSUMER_H_INCLUDED
#define UR_ROBOT_DRIVER_PRIMARY_SHELL_CONSUMER_H_INCLUDED

#include "ur_robot_driver/log.h"
#include "ur_robot_driver/primary/abstract_primary_consumer.h"

namespace ur_driver
{
namespace primary_interface
{
class PrimaryShellConsumer : public AbstractPrimaryConsumer
{
public:
  PrimaryShellConsumer() = default;
  virtual ~PrimaryShellConsumer() = default;

  virtual bool consume(RobotMessage& msg) override
  {
    LOG_INFO("---RobotMessage:---\n%s", msg.toString().c_str());
    return true;
  }
  virtual bool consume(RobotState& msg) override
  {
    LOG_INFO("---RobotState:---\n%s", msg.toString().c_str());
    return true;
  }
  virtual bool consume(ErrorCodeMessage& msg) override
  {
    LOG_INFO("---ErrorCodeMessage---%s", msg.toString().c_str());
    return true;
  }
  virtual bool consume(KeyMessage& msg) override
  {
    LOG_INFO("---KeyMessage---%s", msg.toString().c_str());
    return true;
  }
  virtual bool consume(RuntimeExceptionMessage& msg) override
  {
    LOG_INFO("---RuntimeExceptionMessage---%s", msg.toString().c_str());
    return true;
  }
  virtual bool consume(TextMessage& msg) override
  {
    LOG_INFO("---TextMessage---%s", msg.toString().c_str());
    return true;
  }
  virtual bool consume(VersionMessage& msg) override
  {
    LOG_INFO("---VersionMessage---%s", msg.toString().c_str());
    return true;
  }
  virtual bool consume(KinematicsInfo& msg) override
  {
    LOG_INFO("%s", msg.toString().c_str());
    return true;
  }

private:
  /* data */
};
}  // namespace primary_interface
}  // namespace ur_driver

#endif  // ifndef UR_ROBOT_DRIVER_PRIMARY_SHELL_CONSUMER_H_INCLUDED
