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

#include <ur_robot_driver/comm/pipeline.h>

#include <ur_robot_driver/primary/robot_state/kinematics_info.h>

namespace ur_driver
{
/*!
 * \brief The CalibrationChecker class consumes primary packages ignoring all but KinematicsInfo
 * packages. These are then checked against the used kinematics to see if the correct calibration
 * is used.
 */
class CalibrationChecker : public comm::IConsumer<primary_interface::PrimaryPackage>
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
   * \brief Empty setup function, as no setup is needed.
   */
  virtual void setupConsumer()
  {
  }
  /*!
   * \brief Tears down the consumer.
   */
  virtual void teardownConsumer()
  {
  }
  /*!
   * \brief Stops the consumer.
   */
  virtual void stopConsumer()
  {
  }
  /*!
   * \brief Handles timeouts.
   */
  virtual void onTimeout()
  {
  }

  /*!
   * \brief Consumes a package, checking its hash if it is a KinematicsInfo package. If the hash
   * does not match the expected hash, an error is logged.
   *
   * \param product The package to consume
   *
   * \returns True, if the package was consumed correctly
   */
  virtual bool consume(std::shared_ptr<primary_interface::PrimaryPackage> product);

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
