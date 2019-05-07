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
 * \date    2019-04-10
 *
 */
//----------------------------------------------------------------------

#include "ur_rtde_driver/rtde/rtde_client.h"

namespace ur_driver
{
namespace rtde_interface
{
RTDEClient::RTDEClient(std::string ROBOT_IP, comm::INotifier& notifier)
  : stream_(ROBOT_IP, UR_RTDE_PORT)
  , parser_(readRecipe())
  , prod_(stream_, parser_)
  , pipeline_(prod_, PIPELINE_NAME, notifier)
  , max_frequency_(URE_MAX_FREQUENCY)
{
}

bool RTDEClient::init()
{
  pipeline_.run();
  uint8_t buffer[4096];
  size_t size;
  size_t written;
  // negotiate version
  uint16_t protocol_version = 2;
  size = RequestProtocolVersionRequest::generateSerializedRequest(buffer, protocol_version);
  stream_.write(buffer, size, written);
  std::unique_ptr<comm::URPackage<PackageHeader>> package;
  pipeline_.getLatestProduct(package, std::chrono::milliseconds(1000));
  rtde_interface::RequestProtocolVersion* tmp_version =
      dynamic_cast<rtde_interface::RequestProtocolVersion*>(package.get());
  if (!tmp_version->accepted_)
  {
    protocol_version = 1;
    size = RequestProtocolVersionRequest::generateSerializedRequest(buffer, protocol_version);
    stream_.write(buffer, size, written);
    pipeline_.getLatestProduct(package, std::chrono::milliseconds(1000));
    tmp_version = dynamic_cast<rtde_interface::RequestProtocolVersion*>(package.get());
    if (!tmp_version->accepted_)
    {
      LOG_ERROR("Could not negotiate protocol version");
      return false;
    }
  }

  // determine maximum frequency from ur-control version
  size = GetUrcontrolVersionRequest::generateSerializedRequest(buffer);
  stream_.write(buffer, size, written);
  pipeline_.getLatestProduct(package, std::chrono::milliseconds(1000));
  rtde_interface::GetUrcontrolVersion* tmp_control_version =
      dynamic_cast<rtde_interface::GetUrcontrolVersion*>(package.get());
  if (tmp_control_version->major_ < 5)
  {
    max_frequency_ = CB3_MAX_FREQUENCY;
  }

  // sending output recipe
  LOG_INFO("Setting up RTDE communication with frequency %f", max_frequency_);
  if (protocol_version == 2)
  {
    size = ControlPackageSetupOutputsRequest::generateSerializedRequest(buffer, max_frequency_, readRecipe());
  }
  else
  {
    size = ControlPackageSetupOutputsRequest::generateSerializedRequest(buffer, readRecipe());
  }
  stream_.write(buffer, size, written);
  return pipeline_.getLatestProduct(package, std::chrono::milliseconds(1000));
}
bool RTDEClient::start()
{
  uint8_t buffer[4096];
  size_t size;
  size_t written;
  size = ControlPackageStartRequest::generateSerializedRequest(buffer);
  std::unique_ptr<comm::URPackage<PackageHeader>> package;
  stream_.write(buffer, size, written);
  pipeline_.getLatestProduct(package, std::chrono::milliseconds(1000));
  rtde_interface::ControlPackageStart* tmp = dynamic_cast<rtde_interface::ControlPackageStart*>(package.get());
  return tmp->accepted_;
}
std::vector<std::string> RTDEClient::readRecipe()
{
  std::vector<std::string> recipe;
  recipe.push_back("timestamp");
  recipe.push_back("actual_q");
  recipe.push_back("actual_qd");
  recipe.push_back("speed_scaling");
  recipe.push_back("target_speed_fraction");
  return recipe;
}

bool RTDEClient::getDataPackage(std::unique_ptr<comm::URPackage<PackageHeader>>& data_package,
                                std::chrono::milliseconds timeout)
{
  return pipeline_.getLatestProduct(data_package, timeout);
}
}  // namespace rtde_interface
}  // namespace ur_driver
