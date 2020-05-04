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
 * \date    2019-06-14
 *
 */
//----------------------------------------------------------------------
#ifndef UR_RTDE_DRIVER_UR_CALIBRATION_CHECKER_H_INCLUDED
#define UR_RTDE_DRIVER_UR_CALIBRATION_CHECKER_H_INCLUDED

#include <ur_robot_driver/primary/primary_package_handler.h>
#include <ur_robot_driver/primary/robot_state/kinematics_info.h>

namespace ur_driver
{
/*!
 * \brief The CalibrationChecker checks a received KinematicsInfo package against a registered calibration hash
 * value. This way we know whether the robot that sent the KinematicsInfo package matches the
 * expected calibration.
 */
class CalibrationChecker : public primary_interface::IPrimaryPackageHandler<primary_interface::KinematicsInfo>
{
public:
  /*!
   * \brief Creates a new CalibrationChecker object with an expected hash calculated from the used
   * kinematics.
   *
   * \param expected_hash The expected kinematics hash
   */
  CalibrationChecker(const std::string& expected_hash);
  virtual ~CalibrationChecker() = default;

  /*!
   * \brief Consumes a package, checking its hash if it is a KinematicsInfo package. If the hash
   * does not match the expected hash, an error is logged.
   *
   * \param product The package to consume
   *
   * \returns True, if the package was consumed correctly
   */
  virtual void handle(primary_interface::KinematicsInfo& kin_info) override;

  /*!
   * \brief Used to make sure the calibration check is not performed several times.
   *
   * \returns True, if the calibration was already checked, false otherwise
   */
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
