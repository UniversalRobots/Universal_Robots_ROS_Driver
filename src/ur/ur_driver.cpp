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
 * \author  Felix Mauch mauch@fzi.de
 * \date    2019-04-11
 *
 */
//----------------------------------------------------------------------

#include "ur_rtde_driver/ur/ur_driver.h"
#include <memory>

namespace ur_driver
{
ur_driver::UrDriver::UrDriver(const std::string& ROBOT_IP) : rtde_frequency_(125)  // conservative CB3 default.
{
  ROS_INFO_STREAM("Initializing RTDE client");
  rtde_client_.reset(new rtde_interface::RTDEClient(ROBOT_IP, notifier_));

  if (!rtde_client_->init())
  {
    throw std::runtime_error("initialization went wrong");  // TODO: be less harsh
  }
  rtde_client_->start();  // TODO: Add extra start method (also to HW-Interface)
}

std::unique_ptr<rtde_interface::DataPackage> ur_driver::UrDriver::getDataPackage()
{
  // TODO: This goes into the rtde_client
  std::unique_ptr<comm::URPackage<rtde_interface::PackageHeader>> urpackage;
  uint32_t period_ms = (1.0 / rtde_frequency_) * 1000;
  std::chrono::milliseconds timeout(period_ms);
  if (rtde_client_->getDataPackage(urpackage, timeout))
  {
    rtde_interface::DataPackage* tmp = dynamic_cast<rtde_interface::DataPackage*>(urpackage.get());
    if (tmp != nullptr)
    {
      urpackage.release();
      return std::move(std::unique_ptr<rtde_interface::DataPackage>(tmp));
    }
  }
  return nullptr;
}
}  // namespace ur_driver
