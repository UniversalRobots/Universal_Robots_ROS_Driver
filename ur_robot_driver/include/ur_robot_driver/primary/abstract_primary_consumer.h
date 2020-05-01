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

#ifndef UR_ROBOT_DRIVER_ABSTRACT_PRIMARY_CONSUMER_H_INCLUDED
#define UR_ROBOT_DRIVER_ABSTRACT_PRIMARY_CONSUMER_H_INCLUDED

#include "ur_robot_driver/log.h"
#include "ur_robot_driver/comm/pipeline.h"
#include "ur_robot_driver/primary/robot_message/version_message.h"
#include "ur_robot_driver/primary/robot_state/kinematics_info.h"

namespace ur_driver
{
namespace primary_interface
{
class AbstractPrimaryConsumer : public comm::IConsumer<PrimaryPackage>
{
public:
  AbstractPrimaryConsumer() = default;
  virtual ~AbstractPrimaryConsumer() = default;

  virtual bool consume(std::shared_ptr<PrimaryPackage> product)
  {
    if (product != nullptr)
    {
      return product->consumeWith(*this);
    }
    return false;
  }

  virtual bool consume(PrimaryPackage& pkg) = 0;
  virtual bool consume(VersionMessage& pkg) = 0;
  virtual bool consume(KinematicsInfo& pkg) = 0;

private:
  /* data */
};
}  // namespace primary_interface
}  // namespace ur_driver

#endif  // ifndef UR_ROBOT_DRIVER_ABSTRACT_PRIMARY_CONSUMER_H_INCLUDED

