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

#ifndef UR_ROBOT_DRIVER_PRIMARY_CONSUMER_H_INCLUDED
#define UR_ROBOT_DRIVER_PRIMARY_CONSUMER_H_INCLUDED

#include "ur_robot_driver/log.h"
#include "ur_robot_driver/comm/pipeline.h"
#include "ur_robot_driver/primary/primary_package_handler.h"
#include "ur_robot_driver/primary/abstract_primary_consumer.h"
#include "ur_robot_driver/primary/key_message_handler.h"
#include "ur_robot_driver/primary/robot_message/error_code_message.h"
#include "ur_robot_driver/primary/robot_message/key_message.h"
#include "ur_robot_driver/primary/robot_message/runtime_exception_message.h"
#include "ur_robot_driver/primary/robot_message/text_message.h"
#include "ur_robot_driver/primary/robot_message/version_message.h"
#include "ur_robot_driver/primary/robot_state/kinematics_info.h"

namespace ur_driver
{
namespace primary_interface
{
/*!
 * \brief Primary consumer implementation
 *
 * This class implements am AbstractPrimaryConsumer such that it can consume all incoming primary
 * messages. However, actual work will be done by workers for each specific type.
 */
class PrimaryConsumer : public AbstractPrimaryConsumer
{
public:
  PrimaryConsumer()
  {
    LOG_INFO("Constructing primary consumer");
    key_message_worker_.reset(new KeyMessageHandler());
    LOG_INFO("Constructed primary consumer");
  }
  virtual ~PrimaryConsumer() = default;

  virtual bool consume(RobotMessage& msg) override
  {
    LOG_INFO("---RobotMessage:---\n%s", msg.toString().c_str());
    return true;
  }
  virtual bool consume(RobotState& msg) override
  {
    // LOG_INFO("---RobotState:---\n%s", msg.toString().c_str());
    return true;
  }
  virtual bool consume(ErrorCodeMessage& msg) override
  {
    LOG_INFO("---ErrorCodeMessage---\n%s", msg.toString().c_str());
    return true;
  }
  virtual bool consume(RuntimeExceptionMessage& msg) override
  {
    LOG_INFO("---RuntimeExceptionMessage---\n%s", msg.toString().c_str());
    return true;
  }
  virtual bool consume(TextMessage& msg) override
  {
    LOG_INFO("---TextMessage---\n%s", msg.toString().c_str());
    return true;
  }
  virtual bool consume(VersionMessage& msg) override
  {
    LOG_INFO("---VersionMessage---\n%s", msg.toString().c_str());
    return true;
  }

  /*!
   * \brief Handle a KinematicsInfo
   *
   * \returns True if there's a handler for this message type registered. False otherwise.
   */
  virtual bool consume(KinematicsInfo& pkg) override
  {
    if (kinematics_info_message_worker_ != nullptr)
    {
      kinematics_info_message_worker_->handle(pkg);
      return true;
    }
    return false;
  }

  /*!
   * \brief Handle a KeyMessage
   *
   * \returns True if there's a handler for this message type registered. False otherwise.
   */
  virtual bool consume(KeyMessage& pkg) override
  {
    if (key_message_worker_ != nullptr)
    {
      key_message_worker_->handle(pkg);
      return true;
    }
    return false;
  }

  void setKinematicsInfoHandler(const std::shared_ptr<IPrimaryPackageHandler<KinematicsInfo>>& handler)
  {
    kinematics_info_message_worker_ = handler;
  }

private:
  std::shared_ptr<IPrimaryPackageHandler<KeyMessage>> key_message_worker_;
  std::shared_ptr<IPrimaryPackageHandler<KinematicsInfo>> kinematics_info_message_worker_;
};
}  // namespace primary_interface
}  // namespace ur_driver

#endif  // ifndef UR_ROBOT_DRIVER_PRIMARY_CONSUMER_H_INCLUDED
