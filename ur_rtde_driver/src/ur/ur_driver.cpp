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
//
// Many parts from this (Most of the URScript program) comes from the ur_modern_driver
// Copyright 2017, 2018 Simon Rasmussen (refactor)
// Copyright 2015, 2016 Thomas Timm Andersen (original version)
//
// -- END LICENSE BLOCK ------------------------------------------------

//----------------------------------------------------------------------
/*!\file
 *
 * \author  Felix Mauch mauch@fzi.de
 * \date    2019-04-11
 *
 */
//----------------------------------------------------------------------

#include "ur_rtde_driver/ur/ur_driver.h"
#include "ur_rtde_driver/primary/package_header.h"
#include <memory>

namespace ur_driver
{
static const int32_t MULT_JOINTSTATE_ = 1000000;
static const std::string JOINT_STATE_REPLACE("{{JOINT_STATE_REPLACE}}");
static const std::string SERVO_J_REPLACE("{{SERVO_J_REPLACE}}");
static const std::string SERVER_IP_REPLACE("{{SERVER_IP_REPLACE}}");
static const std::string SERVER_PORT_REPLACE("{{SERVER_PORT_REPLACE}}");

ur_driver::UrDriver::UrDriver(const std::string& robot_ip, const std::string& script_file)
  : servoj_time_(0.008), servoj_gain_(750), servoj_lookahead_time_(0.03)
{
  LOG_INFO("Initializing RTDE client");
  rtde_client_.reset(new rtde_interface::RTDEClient(robot_ip, notifier_));

  if (!rtde_client_->init())
  {
    throw std::runtime_error("initialization went wrong");  // TODO: be less harsh
  }

  rtde_frequency_ = rtde_client_->getMaxFrequency();
  servoj_time_ = 1.0 / rtde_frequency_;

  // Open Stream to get own IP
  // TODO: Open Primary interface to query version and calibration
  comm::URStream<primary_interface::PackageHeader> stream(robot_ip, 30001);
  stream.connect();
  std::string local_ip = stream.getIP();

  uint32_t reverse_port = 50001;  // TODO: Make this a parameter

  std::string prog = readScriptFile(script_file);
  prog.replace(prog.find(JOINT_STATE_REPLACE), JOINT_STATE_REPLACE.length(), std::to_string(MULT_JOINTSTATE_));
  std::ostringstream out;
  out << "lookahead_time=" << servoj_lookahead_time_ << ", gain=" << servoj_gain_;
  prog.replace(prog.find(SERVO_J_REPLACE), SERVO_J_REPLACE.length(), out.str());
  prog.replace(prog.find(SERVO_J_REPLACE), SERVO_J_REPLACE.length(), out.str());
  prog.replace(prog.find(SERVER_IP_REPLACE), SERVER_IP_REPLACE.length(), local_ip);
  prog.replace(prog.find(SERVER_PORT_REPLACE), SERVER_PORT_REPLACE.length(), std::to_string(reverse_port));
  size_t len = prog.size();
  const uint8_t* data = reinterpret_cast<const uint8_t*>(prog.c_str());
  size_t written;

  if (stream.write(data, len, written))
  {
    LOG_INFO("Sent program to robot");
  }
  else
  {
    LOG_ERROR("Could not send program to robot");
  }

  reverse_interface_.reset(new comm::ReverseInterface(reverse_port));

  LOG_INFO("Created reverse interface");

  rtde_client_->start();  // TODO: Add extra start method (also to HW-Interface)
  LOG_INFO("Initialization done");
}

std::unique_ptr<rtde_interface::DataPackage> ur_driver::UrDriver::getDataPackage()
{
  // TODO: This goes into the rtde_client
  std::unique_ptr<comm::URPackage<rtde_interface::PackageHeader>> urpackage;
  std::chrono::milliseconds timeout(100);  // We deliberately have a quite large timeout here, as the robot itself
                                           // should command the control loop's timing.
  if (rtde_client_->getDataPackage(urpackage, timeout))
  {
    rtde_interface::DataPackage* tmp = dynamic_cast<rtde_interface::DataPackage*>(urpackage.get());
    if (tmp != nullptr)
    {
      urpackage.release();
      return std::unique_ptr<rtde_interface::DataPackage>(tmp);
    }
  }
  return nullptr;
}

bool UrDriver::writeJointCommand(const vector6d_t& values)
{
  if (reverse_interface_)
  {
    reverse_interface_->write(values);
  }
  else
  {
    return false;
  }

  return true;
}

std::string UrDriver::readScriptFile(const std::string& filename)
{
  std::ifstream ifs(filename);
  std::string content((std::istreambuf_iterator<char>(ifs)), (std::istreambuf_iterator<char>()));

  return content;
}
}  // namespace ur_driver
