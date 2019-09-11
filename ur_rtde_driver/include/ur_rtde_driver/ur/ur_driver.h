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
#ifndef UR_RTDE_DRIVER_UR_UR_DRIVER_H_INCLUDED
#define UR_RTDE_DRIVER_UR_UR_DRIVER_H_INCLUDED

#include "ur_rtde_driver/rtde/rtde_client.h"
#include "ur_rtde_driver/comm/reverse_interface.h"
#include "ur_rtde_driver/comm/script_sender.h"
#include "ur_rtde_driver/ur/tool_communication.h"
#include "ur_rtde_driver/primary/robot_message/version_message.h"
#include "ur_rtde_driver/rtde/rtde_writer.h"

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
   * \param robot_ip IP-address under which the robot is reachable.
   * \param script_file URScript file that should be sent to the robot
   * \param tool_comm_setup Configuration for using the tool communication
   */
  UrDriver(const std::string& robot_ip, const std::string& script_file, const std::string& output_recipe_file,
           const std::string& input_recipe_file, std::function<void(bool)> handle_program_state,
           std::unique_ptr<ToolCommSetup> tool_comm_setup, const std::string& calibration_checksum = "");
  /*!
   * \brief Constructs a new UrDriver object.
   *
   * \param robot_ip IP-address under which the robot is reachable.
   * \param script_file URScript file that should be sent to the robot
   */
  UrDriver(const std::string& robot_ip, const std::string& script_file, const std::string& output_recipe_file,
           const std::string& input_recipe_file, std::function<void(bool)> handle_program_state,
           const std::string& calibration_checksum = "")
    : UrDriver(robot_ip, script_file, output_recipe_file, input_recipe_file, handle_program_state,
               std::unique_ptr<ToolCommSetup>{}, calibration_checksum)
  {
  }

  virtual ~UrDriver() = default;

  /*!
   * \brief Access function to receive the latest data package sent from the robot through RTDE
   * interface.
   *
   * \returns The latest data package on success, a nullptr if no package can be found inside the
   * interface's cycle time. See the private parameter #rtde_frequency_
   */
  std::unique_ptr<rtde_interface::DataPackage> getDataPackage();

  uint32_t getControlFrequency() const
  {
    return rtde_frequency_;
  }

  /*!
   * \brief Writes a joint command together with a keepalive signal onto the socket being sent to
   * the robot.
   *
   * \param values Desired joint positions
   *
   * \returns True on successful write.
   */
  bool writeJointCommand(const vector6d_t& values);

  /*!
   * \brief Write a keepalive signal only.
   *
   * This signals the robot that the connection is still
   * active in times when no commands are to be sent (e.g. no controller is active.)
   *
   * \returns True on successful write.
   */
  bool writeKeepalive();

  /*!
   * \brief Sends a stop command to the socket interface which will signal the program running on
   * the robot to no longer listen for commands sent from the remote pc.
   *
   * \returns True on successful write.
   */
  bool stopControl();

  void startWatchdog();

  void checkCalibration(const std::string& checksum);

  rtde_interface::RTDEWriter& getRTDEWriter();

  /*!
   * \brief Sends a custom script program to the robot.
   *
   * The given code must be valid according the UR Scripting Manual.
   *
   * \param program URScript code that shall be executed by the robot.
   *
   * \returns true on successful upload, false otherwise.
   */
  bool sendScript(const std::string& program);

private:
  std::string readScriptFile(const std::string& filename);
  std::string readKeepalive();

  int rtde_frequency_;
  comm::INotifier notifier_;
  std::unique_ptr<rtde_interface::RTDEClient> rtde_client_;
  std::unique_ptr<comm::ReverseInterface> reverse_interface_;
  std::unique_ptr<comm::ScriptSender> script_sender_;
  std::unique_ptr<comm::URStream<ur_driver::primary_interface::PackageHeader>> primary_stream_;
  std::unique_ptr<comm::URStream<ur_driver::primary_interface::PackageHeader>> secondary_stream_;

  double servoj_time_;
  uint32_t servoj_gain_;
  double servoj_lookahead_time_;

  std::thread watchdog_thread_;
  bool reverse_interface_active_;
  uint32_t reverse_port_;
  std::function<void(bool)> handle_program_state_;

  std::string robot_ip_;
};
}  // namespace ur_driver
#endif  // ifndef UR_RTDE_DRIVER_UR_UR_DRIVER_H_INCLUDED
