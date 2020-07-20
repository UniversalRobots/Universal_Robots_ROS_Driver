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

  uint16_t protocol_version = MAX_RTDE_PROTOCOL_VERSION;
  while (!negotiateProtocolVersion(protocol_version))
  {
    LOG_INFO("Robot did not accept RTDE protocol version '%hu'. Trying lower protocol version", protocol_version);
    protocol_version--;
    if (protocol_version == 0)
    {
      throw UrException("Protocol version for RTDE communication could not be established. Robot didn't accept any of "
                        "the suggested versions.");
    }
  }
  LOG_INFO("Negotiated RTDE protocol version to %hu.", protocol_version);
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

bool RTDEClient::negotiateProtocolVersion(const uint16_t protocol_version)
{
  static unsigned num_retries = 0;
  uint8_t buffer[4096];
  size_t size;
  size_t written;
  size = RequestProtocolVersionRequest::generateSerializedRequest(buffer, protocol_version);
  if (!stream_.write(buffer, size, written))
    throw UrException("Sending protocol version query to robot failed.");

  std::unique_ptr<RTDEPackage> package;
  while (num_retries < MAX_REQUEST_RETRIES)
  {
    if (!pipeline_.getLatestProduct(package, std::chrono::milliseconds(1000)))
    {
      throw UrException("No answer to RTDE protocol version negotiation request was received from robot. This should "
                        "not "
                        "happen!");
    }

    if (rtde_interface::RequestProtocolVersion* tmp_version =
            dynamic_cast<rtde_interface::RequestProtocolVersion*>(package.get()))
    {
      // Reset the num_tries variable in case we have to try with another protocol version.
      num_retries = 0;
      return tmp_version->accepted_;
    }
    else
    {
      std::stringstream ss;
      ss << "Did not receive protocol negotiation answer from robot. Message received instead: " << std::endl
         << package->toString() << ". Retrying...";
      num_retries++;
      LOG_WARN("%s", ss.str().c_str());
    }
  }
  std::stringstream ss;
  ss << "Could not negotiate RTDE protocol version after " << MAX_REQUEST_RETRIES
     << " tries. Please check the output of the "
        "negotiation attempts above to get a hint what could be wrong.";
  throw UrException(ss.str());
}

void RTDEClient::queryURControlVersion()
{
  static unsigned num_retries = 0;
  uint8_t buffer[4096];
  size_t size;
  size_t written;
  size = GetUrcontrolVersionRequest::generateSerializedRequest(buffer);
  if (!stream_.write(buffer, size, written))
    throw UrException("Sending urcontrol version query request to robot failed.");

  std::unique_ptr<RTDEPackage> package;
  while (num_retries < MAX_REQUEST_RETRIES)
  {
    if (!pipeline_.getLatestProduct(package, std::chrono::milliseconds(1000)))
      throw UrException("No answer to urcontrol version query was received from robot. This should not happen!");

    if (rtde_interface::GetUrcontrolVersion* tmp_urcontrol_version =
            dynamic_cast<rtde_interface::GetUrcontrolVersion*>(package.get()))
    {
      urcontrol_version_ = tmp_urcontrol_version->version_information_;
      return;
    }
    else
    {
      std::stringstream ss;
      ss << "Did not receive protocol negotiation answer from robot. Message received instead: " << std::endl
         << package->toString() << ". Retrying...";
      num_retries++;
      LOG_WARN("%s", ss.str().c_str());
    }
  }
  std::stringstream ss;
  ss << "Could not query urcontrol version after " << MAX_REQUEST_RETRIES
     << " tries. Please check the output of the "
        "negotiation attempts above to get a hint what could be wrong.";
  throw UrException(ss.str());
}

void RTDEClient::setupOutputs(const uint16_t protocol_version)
{
  static unsigned num_retries = 0;
  size_t size;
  size_t written;
  uint8_t buffer[4096];
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

  std::unique_ptr<RTDEPackage> package;
  while (num_retries < MAX_REQUEST_RETRIES)
  {
    if (!pipeline_.getLatestProduct(package, std::chrono::milliseconds(1000)))
    {
      throw UrException("Did not receive confirmation on RTDE output recipe.");
    }

    if (rtde_interface::ControlPackageSetupOutputs* tmp_output =
            dynamic_cast<rtde_interface::ControlPackageSetupOutputs*>(package.get()))

    {
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
      return;
    }
    else
    {
      std::stringstream ss;
      ss << "Did not receive answer to RTDE output setup. Message received instead: " << std::endl
         << package->toString() << ". Retrying...";
      num_retries++;
      LOG_WARN("%s", ss.str().c_str());
    }
  }
  std::stringstream ss;
  ss << "Could not setup RTDE outputs after " << MAX_REQUEST_RETRIES
     << " tries. Please check the output of the "
        "negotiation attempts above to get a hint what could be wrong.";
  throw UrException(ss.str());
}

void RTDEClient::setupInputs()
{
  static unsigned num_retries = 0;
  size_t size;
  size_t written;
  uint8_t buffer[4096];
  size = ControlPackageSetupInputsRequest::generateSerializedRequest(buffer, input_recipe_);
  if (!stream_.write(buffer, size, written))
    throw UrException("Could not send RTDE input recipe to robot.");

  std::unique_ptr<RTDEPackage> package;
  while (num_retries < MAX_REQUEST_RETRIES)
  {
    if (!pipeline_.getLatestProduct(package, std::chrono::milliseconds(1000)))
      throw UrException("Did not receive confirmation on RTDE input recipe.");

    if (rtde_interface::ControlPackageSetupInputs* tmp_input =
            dynamic_cast<rtde_interface::ControlPackageSetupInputs*>(package.get()))

    {
      std::vector<std::string> variable_types = splitVariableTypes(tmp_input->variable_types_);
      assert(input_recipe_.size() == variable_types.size());
      for (std::size_t i = 0; i < variable_types.size(); ++i)
      {
        LOG_DEBUG("%s confirmed as datatype: %s", input_recipe_[i].c_str(), variable_types[i].c_str());
        if (variable_types[i] == "NOT_FOUND")
        {
          std::string message = "Variable '" + input_recipe_[i] +
                                "' not recognized by the robot. Probably your input recipe contains errors";
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

      return;
    }
    else
    {
      std::stringstream ss;
      ss << "Did not receive answer to RTDE input setup. Message received instead: " << std::endl
         << package->toString() << ". Retrying...";
      num_retries++;
      LOG_WARN("%s", ss.str().c_str());
    }
  }
  std::stringstream ss;
  ss << "Could not setup RTDE inputs after " << MAX_REQUEST_RETRIES
     << " tries. Please check the output of the "
        "negotiation attempts above to get a hint what could be wrong.";
  throw UrException(ss.str());
}

bool RTDEClient::start()
{
  static unsigned num_retries = 0;
  uint8_t buffer[4096];
  size_t size;
  size_t written;
  pipeline_.run();
  size = ControlPackageStartRequest::generateSerializedRequest(buffer);
  std::unique_ptr<RTDEPackage> package;
  if (!stream_.write(buffer, size, written))
    throw UrException("Sending RTDE start command failed!");
  while (num_retries < MAX_REQUEST_RETRIES)
  {
    if (!pipeline_.getLatestProduct(package, std::chrono::milliseconds(1000)))
      throw UrException("Could not get response to RTDE communication start request from robot. This should not "
                        "happen!");
    if (rtde_interface::ControlPackageStart* tmp = dynamic_cast<rtde_interface::ControlPackageStart*>(package.get()))
    {
      return tmp->accepted_;
    }
    else
    {
      std::stringstream ss;
      ss << "Did not receive answer to RTDE start request. Message received instead: " << std::endl
         << package->toString() << ". Retrying...";
      LOG_WARN("%s", ss.str().c_str());
    }
  }
  std::stringstream ss;
  ss << "Could not start RTDE communication after " << MAX_REQUEST_RETRIES
     << " tries. Please check the output of the "
        "negotiation attempts above to get a hint what could be wrong.";
  throw UrException(ss.str());
}

std::vector<std::string> RTDEClient::readRecipe(const std::string& recipe_file)
{
  std::vector<std::string> recipe;
  std::ifstream file(recipe_file);
  if (file.fail())
  {
    std::stringstream msg;
    msg << "Opening file '" << recipe_file << "' failed with error: " << strerror(errno);
    LOG_ERROR("%s", msg.str().c_str());
    throw UrException(msg.str());
  }
  std::string line;
  while (std::getline(file, line))
  {
    recipe.push_back(line);
  }
  return recipe;
}

std::unique_ptr<rtde_interface::DataPackage> RTDEClient::getDataPackage(std::chrono::milliseconds timeout)
{
  std::unique_ptr<RTDEPackage> urpackage;
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
