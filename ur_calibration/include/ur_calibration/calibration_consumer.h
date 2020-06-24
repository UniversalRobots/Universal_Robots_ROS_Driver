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

#ifndef UR_CALIBRATION_CALIBRATION_CONSUMER_H_INCLUDED
#define UR_CALIBRATION_CALIBRATION_CONSUMER_H_INCLUDED
#include <ur_robot_driver/comm/pipeline.h>

#include <ur_robot_driver/primary/robot_state/kinematics_info.h>

#include <ur_calibration/calibration.h>

namespace ur_calibration
{
class CalibrationConsumer : public ur_driver::comm::IConsumer<ur_driver::primary_interface::PrimaryPackage>
{
public:
  CalibrationConsumer();
  virtual ~CalibrationConsumer() = default;

  virtual bool consume(std::shared_ptr<ur_driver::primary_interface::PrimaryPackage> product);

  bool isCalibrated() const
  {
    return calibrated_;
  }

  YAML::Node getCalibrationParameters() const;

private:
  bool calibrated_;
  YAML::Node calibration_parameters_;
};
}  // namespace ur_calibration
#endif  // ifndef UR_CALIBRATION_CALIBRATION_CONSUMER_H_INCLUDED
