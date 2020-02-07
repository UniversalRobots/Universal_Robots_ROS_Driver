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

#include "ur_robot_driver/rtde/rtde_client.h"
#include "ur_robot_driver/exceptions.h"

namespace ur_driver
{
namespace rtde_interface
{
RTDEClient::RTDEClient(std::string robot_ip, comm::INotifier& notifier, const std::string& output_recipe_file,
                       const std::string& input_recipe_file)
  : stream_(robot_ip, UR_RTDE_PORT)
  , output_recipe_(readRecipe(output_recipe_file))
  , input_recipe_(readRecipe(input_recipe_file))
  , parser_(output_recipe_)
  , prod_(stream_, parser_)
  , pipeline_(prod_, PIPELINE_NAME, notifier)
  , writer_(&stream_, input_recipe_)
  , max_frequency_(URE_MAX_FREQUENCY)
{
}

bool RTDEClient::init()
{
  // A running pipeline is needed inside setup
  pipeline_.init();
  pipeline_.run();

  uint16_t protocol_version = negotiateProtocolVersion();
  parser_.setProtocolVersion(protocol_version);

  queryURControlVersion();
  if (urcontrol_version_.major < 5)
  {
    max_frequency_ = CB3_MAX_FREQUENCY;
  }

  setupOutputs(protocol_version);
  setupInputs();

  // We finished communication for now
  pipeline_.stop();

  // We throw exceptions on the way, so if we made it that far, we can return true.
  return true;
}

uint16_t RTDEClient::negotiateProtocolVersion()
{
  uint8_t buffer[4096];
  size_t size;
  size_t written;
  uint16_t protocol_version = 2;
  size = RequestProtocolVersionRequest::generateSerializedRequest(buffer, protocol_version);
  if (!stream_.write(buffer, size, written))
    throw UrException("Sending protocol version query to robot failed.");
  std::unique_ptr<comm::URPackage<PackageHeader>> package;
  if (!pipeline_.getLatestProduct(package, std::chrono::milliseconds(1000)))
    throw UrException("Could not get urcontrol version from robot. This should not happen!");
  rtde_interface::RequestProtocolVersion* tmp_version =
      dynamic_cast<rtde_interface::RequestProtocolVersion*>(package.get());
  if (!tmp_version->accepted_)
  {
    protocol_version = 1;
    size = RequestProtocolVersionRequest::generateSerializedRequest(buffer, protocol_version);
    if (!stream_.write(buffer, size, written))
      throw UrException("Sending protocol version query to robot failed.");
    if (!pipeline_.getLatestProduct(package, std::chrono::milliseconds(1000)))
      throw UrException("Could not get urcontrol version from robot. This should not happen!");
    tmp_version = dynamic_cast<rtde_interface::RequestProtocolVersion*>(package.get());
    if (!tmp_version->accepted_)
    {
      throw UrException("Neither protocol version 1 nor 2 was accepted by the robot. This should not happen!");
    }
  }
  return protocol_version;
}

void RTDEClient::queryURControlVersion()
{
  uint8_t buffer[4096];
  size_t size;
  size_t written;
  std::unique_ptr<comm::URPackage<PackageHeader>> package;
  size = GetUrcontrolVersionRequest::generateSerializedRequest(buffer);
  if (!stream_.write(buffer, size, written))
    throw UrException("Sending urcontrol version query request to robot failed.");
  if (!pipeline_.getLatestProduct(package, std::chrono::milliseconds(1000)))
    throw UrException("Could not get urcontrol version from robot. This should not happen!");
  rtde_interface::GetUrcontrolVersion* tmp_urcontrol_version =
      dynamic_cast<rtde_interface::GetUrcontrolVersion*>(package.get());

  if (tmp_urcontrol_version == nullptr)
  {
    throw UrException("Could not get urcontrol version from robot. This should not happen!");
  }
  urcontrol_version_ = tmp_urcontrol_version->version_information_;
}

void RTDEClient::setupOutputs(const uint16_t protocol_version)
{
  size_t size;
  size_t written;
  uint8_t buffer[4096];
  std::unique_ptr<comm::URPackage<PackageHeader>> package;
  LOG_INFO("Setting up RTDE communication with frequency %f", max_frequency_);
  if (protocol_version == 2)
  {
    size = ControlPackageSetupOutputsRequest::generateSerializedRequest(buffer, max_frequency_, output_recipe_);
  }
  else
  {
    size = ControlPackageSetupOutputsRequest::generateSerializedRequest(buffer, output_recipe_);
  }

  // Send output recipe to robot
  if (!stream_.write(buffer, size, written))
    throw UrException("Could not send RTDE output recipe to robot.");
  if (!pipeline_.getLatestProduct(package, std::chrono::milliseconds(1000)))
  {
    throw UrException("Did not receive confirmation on RTDE output recipe.");
  }

  rtde_interface::ControlPackageSetupOutputs* tmp_output =
      dynamic_cast<rtde_interface::ControlPackageSetupOutputs*>(package.get());

  std::vector<std::string> variable_types = splitVariableTypes(tmp_output->variable_types_);
  assert(output_recipe_.size() == variable_types.size());
  for (std::size_t i = 0; i < variable_types.size(); ++i)
  {
    LOG_DEBUG("%s confirmed as datatype: %s", output_recipe_[i].c_str(), variable_types[i].c_str());
    if (variable_types[i] == "NOT_FOUND")
    {
      std::string message = "Variable '" + output_recipe_[i] +
                            "' not recognized by the robot. Probably your output recipe contains errors";
      throw UrException(message);
    }
  }
}

void RTDEClient::setupInputs()
{
  size_t size;
  size_t written;
  uint8_t buffer[4096];
  std::unique_ptr<comm::URPackage<PackageHeader>> package;
  size = ControlPackageSetupInputsRequest::generateSerializedRequest(buffer, input_recipe_);
  if (!stream_.write(buffer, size, written))
    throw UrException("Could not send RTDE input recipe to robot.");
  if (!pipeline_.getLatestProduct(package, std::chrono::milliseconds(1000)))
    throw UrException("Did not receive confirmation on RTDE input recipe.");
  rtde_interface::ControlPackageSetupInputs* tmp_input =
      dynamic_cast<rtde_interface::ControlPackageSetupInputs*>(package.get());
  if (tmp_input == nullptr)
  {
    throw UrException("Could not setup RTDE inputs.");
  }

  std::vector<std::string> variable_types = splitVariableTypes(tmp_input->variable_types_);
  assert(input_recipe_.size() == variable_types.size());
  for (std::size_t i = 0; i < variable_types.size(); ++i)
  {
    LOG_DEBUG("%s confirmed as datatype: %s", input_recipe_[i].c_str(), variable_types[i].c_str());
    if (variable_types[i] == "NOT_FOUND")
    {
      std::string message =
          "Variable '" + input_recipe_[i] + "' not recognized by the robot. Probably your input recipe contains errors";
      throw UrException(message);
    }
    else if (variable_types[i] == "IN_USE")
    {
      std::string message = "Variable '" + input_recipe_[i] +
                            "' is currently controlled by another RTDE client. The input recipe can't be used as "
                            "configured";
      throw UrException(message);
    }
  }

  writer_.init(tmp_input->input_recipe_id_);
}

bool RTDEClient::start()
{
  uint8_t buffer[4096];
  size_t size;
  size_t written;
  pipeline_.run();
  size = ControlPackageStartRequest::generateSerializedRequest(buffer);
  std::unique_ptr<comm::URPackage<PackageHeader>> package;
  if (!stream_.write(buffer, size, written))
    throw UrException("Sending RTDE start command failed!");
  if (!pipeline_.getLatestProduct(package, std::chrono::milliseconds(1000)))
    throw UrException("Could not get response to RTDE communication start request from robot. This should not "
                      "happen!");
  rtde_interface::ControlPackageStart* tmp = dynamic_cast<rtde_interface::ControlPackageStart*>(package.get());
  return tmp->accepted_;
}
std::vector<std::string> RTDEClient::readRecipe(const std::string& recipe_file)
{
  std::vector<std::string> recipe;
  std::ifstream file(recipe_file);
  std::string line;
  while (std::getline(file, line))
  {
    recipe.push_back(line);
  }
  return recipe;
}

std::unique_ptr<rtde_interface::DataPackage> RTDEClient::getDataPackage(std::chrono::milliseconds timeout)
{
  std::unique_ptr<comm::URPackage<rtde_interface::PackageHeader>> urpackage;
  if (pipeline_.getLatestProduct(urpackage, timeout))
  {
    rtde_interface::DataPackage* tmp = dynamic_cast<rtde_interface::DataPackage*>(urpackage.get());
    if (tmp != nullptr)
    {
      urpackage.release();
      return std::unique_ptr<rtde_interface::DataPackage>(tmp);
    }
  }
  return std::unique_ptr<rtde_interface::DataPackage>(nullptr);
}

std::string RTDEClient::getIP() const
{
  return stream_.getIP();
}

RTDEWriter& RTDEClient::getWriter()
{
  return writer_;
}

std::vector<std::string> RTDEClient::splitVariableTypes(const std::string& variable_types) const
{
  std::vector<std::string> result;
  std::stringstream ss(variable_types);
  std::string substr = "";
  while (getline(ss, substr, ','))
  {
    result.push_back(substr);
  }
  return result;
}
}  // namespace rtde_interface
}  // namespace ur_driver
