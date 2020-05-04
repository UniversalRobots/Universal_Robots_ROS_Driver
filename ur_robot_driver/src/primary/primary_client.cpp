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

#include <chrono>
#include <thread>

#include <ur_robot_driver/primary/primary_client.h>
#include <ur_robot_driver/primary/primary_shell_consumer.h>

namespace ur_driver
{
namespace primary_interface
{
PrimaryClient::PrimaryClient(const std::string& robot_ip, const std::string& calibration_checksum) : robot_ip_(robot_ip)
{
  stream_.reset(new comm::URStream<PrimaryPackage>(robot_ip_, UR_PRIMARY_PORT));
  producer_.reset(new comm::URProducer<PrimaryPackage>(*stream_, parser_));
  producer_->setupProducer();

  consumer_.reset(new PrimaryConsumer());
  std::shared_ptr<CalibrationChecker> calibration_checker(new CalibrationChecker(calibration_checksum));
  consumer_->setKinematicsInfoHandler(calibration_checker);

  pipeline_.reset(new comm::Pipeline<PrimaryPackage>(*producer_, consumer_.get(), "primary pipeline", notifier_));
  pipeline_->run();

  calibration_checker->isChecked();
  while (!calibration_checker->isChecked())
  {
    std::this_thread::sleep_for(std::chrono::seconds(1));
  }
  LOG_DEBUG("Got calibration information from robot.");
}

bool PrimaryClient::sendScript(const std::string& script_code)
{
  if (stream_ == nullptr)
  {
    throw std::runtime_error("Sending script to robot requested while there is no primary interface established. This "
                             "should not happen.");
  }

  // urscripts (snippets) must end with a newline, or otherwise the controller's runtime will
  // not execute them. To avoid problems, we always just append a newline here, even if
  // there may already be one.
  auto program_with_newline = script_code + '\n';

  size_t len = program_with_newline.size();
  const uint8_t* data = reinterpret_cast<const uint8_t*>(program_with_newline.c_str());
  size_t written;

  if (stream_->write(data, len, written))
  {
    LOG_INFO("Sent program to robot:\n%s", program_with_newline.c_str());
    return true;
  }
  LOG_ERROR("Could not send program to robot");
  return false;
}

void PrimaryClient::checkCalibration(const std::string& checksum)
{
  // if (stream_ == nullptr)
  //{
  // throw std::runtime_error("checkCalibration() called without a primary interface connection being established.");
  //}

  // while (!consumer.isChecked())
  //{
  // ros::Duration(1).sleep();
  //}
  // ROS_DEBUG_STREAM("Got calibration information from robot.");
}
}  // namespace primary_interface
}  // namespace ur_driver
