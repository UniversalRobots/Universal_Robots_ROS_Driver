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

namespace ur_driver
{
namespace primary_interface
{
class PrimaryClient
{
public:
  PrimaryClient() = delete;
  PrimaryClient(const std::string& robot_ip);
  virtual ~PrimaryClient() = default;

private:
  std::string robot_ip_;
  PrimaryParser parser_;
  std::unique_ptr<comm::IConsumer<PrimaryPackage>> consumer_;
  comm::INotifier notifier_;
  std::unique_ptr<comm::URProducer<PrimaryPackage>> producer_;
  std::unique_ptr<comm::URStream<PrimaryPackage>> stream_;
  std::unique_ptr<comm::Pipeline<PrimaryPackage>> pipeline_;
};

}  // namespace primary_interface
}  // namespace ur_driver

#endif  // ifndef UR_ROBOT_DRIVER_PRIMARY_CLIENT_H_INCLUDED
