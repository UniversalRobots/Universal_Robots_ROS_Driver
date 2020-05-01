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

#include <ur_robot_driver/primary/primary_client.h>
#include <ur_robot_driver/primary/primary_shell_consumer.h>

namespace ur_driver
{
namespace primary_interface
{
PrimaryClient::PrimaryClient(const std::string& robot_ip) : robot_ip_(robot_ip)
{
  stream_.reset(new comm::URStream<PrimaryPackage>(robot_ip_, UR_PRIMARY_PORT));
  producer_.reset(new comm::URProducer<PrimaryPackage>(*stream_, parser_));
  producer_->setupProducer();

  consumer_.reset(new PrimaryShellConsumer());

  pipeline_.reset(new comm::Pipeline<PrimaryPackage>(*producer_, consumer_.get(), "primary pipeline", notifier_));
  pipeline_->run();
}
}  // namespace primary_interface
}  // namespace ur_driver
