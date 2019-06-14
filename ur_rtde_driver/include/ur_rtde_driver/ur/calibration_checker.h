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
 * \date    2019-06-14
 *
 */
//----------------------------------------------------------------------
#ifndef UR_RTDE_DRIVER_UR_CALIBRATION_CHECKER_H_INCLUDED
#define UR_RTDE_DRIVER_UR_CALIBRATION_CHECKER_H_INCLUDED

#include <ur_rtde_driver/comm/pipeline.h>

#include <ur_rtde_driver/primary/robot_state/kinematics_info.h>

namespace ur_driver
{
class CalibrationChecker : public comm::IConsumer<comm::URPackage<primary_interface::PackageHeader>>
{
public:
  CalibrationChecker(const std::string& expected_hash);
  virtual ~CalibrationChecker() = default;

  virtual void setupConsumer()
  {
  }
  virtual void teardownConsumer()
  {
  }
  virtual void stopConsumer()
  {
  }
  virtual void onTimeout()
  {
  }

  virtual bool consume(std::shared_ptr<comm::URPackage<primary_interface::PackageHeader>> product);

  bool isChecked()
  {
    return checked_;
  }

private:
  std::string expected_hash_;
  bool checked_;
};
}  // namespace ur_driver

#endif  // ifndef UR_RTDE_DRIVER_UR_CALIBRATION_CHECKER_H_INCLUDED
