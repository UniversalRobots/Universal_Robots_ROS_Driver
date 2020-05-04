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
 * \author  Felix Exner exner@fzi.de
 * \date    2019-05-28
 *
 */
//----------------------------------------------------------------------

#include <ur_calibration/calibration_consumer.h>

namespace ur_calibration
{
CalibrationConsumer::CalibrationConsumer() : calibrated_(false)
{
}

bool CalibrationConsumer::consume(std::shared_ptr<ur_driver::primary_interface::PrimaryPackage> product)
{
  auto kin_info = std::dynamic_pointer_cast<ur_driver::primary_interface::KinematicsInfo>(product);
  if (kin_info != nullptr)
  {
    LOG_INFO("%s", product->toString().c_str());
    DHRobot my_robot;
    for (size_t i = 0; i < kin_info->dh_a_.size(); ++i)
    {
      my_robot.segments_.push_back(
          DHSegment(kin_info->dh_d_[i], kin_info->dh_a_[i], kin_info->dh_theta_[i], kin_info->dh_alpha_[i]));
    }
    Calibration calibration(my_robot);
    calibration.correctChain();

    calibration_parameters_ = calibration.toYaml();
    calibration_parameters_["kinematics"]["hash"] = kin_info->toHash();
    calibrated_ = true;
  }
  return true;
}

YAML::Node CalibrationConsumer::getCalibrationParameters() const
{
  if (!calibrated_)
  {
    throw(std::runtime_error("Cannot get calibration, as no calibration data received yet"));
  }
  return calibration_parameters_;
}
}  // namespace ur_calibration
