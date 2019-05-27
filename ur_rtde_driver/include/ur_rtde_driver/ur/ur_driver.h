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

#include "ur_rtde_driver/rtde/rtde_client.h"
#include "ur_rtde_driver/comm/reverse_interface.h"

namespace ur_driver
{
/*!
 * \brief This is the main class for interfacing the driver.
 *
 * It sets up all the necessary socket connections and handles the data exchange with the robot.
 * Use this classes methods to access and write data.
 */
class UrDriver
{
public:
  /*!
   * \brief Constructs a new UrDriver object.
   *
   * \param ROBOT_IP IP-address under which the robot is reachable.
   */
  UrDriver(const std::string& ROBOT_IP);
  virtual ~UrDriver() = default;

  /*!
   * \brief Access function to receive the latest data package sent from the robot through RTDE
   * interface.
   *
   * \returns The latest data package on success, a nullptr if no package can be found inside the
   * interface's cycle time. See the private parameter #rtde_frequency_
   */
  std::unique_ptr<rtde_interface::DataPackage> getDataPackage();

  const uint32_t& getControlFrequency() const
  {
    return rtde_frequency_;
  }

  bool writeJointCommand(const vector6d_t& values);

private:
  //! This frequency is used for the rtde interface. By default, it will be 125Hz on CB3 and 500Hz on the e-series.
  uint32_t rtde_frequency_;
  comm::INotifier notifier_;
  std::unique_ptr<rtde_interface::RTDEClient> rtde_client_;
  std::unique_ptr<comm::ReverseInterface> reverse_interface_;
};
}  // namespace ur_driver
