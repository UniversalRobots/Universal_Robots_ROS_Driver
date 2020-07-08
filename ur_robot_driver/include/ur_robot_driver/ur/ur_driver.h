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
 * \date    2019-04-11
 *
 */
//----------------------------------------------------------------------
#ifndef UR_RTDE_DRIVER_UR_UR_DRIVER_H_INCLUDED
#define UR_RTDE_DRIVER_UR_UR_DRIVER_H_INCLUDED

#include "ur_robot_driver/rtde/rtde_client.h"
#include "ur_robot_driver/comm/reverse_interface.h"
#include "ur_robot_driver/comm/script_sender.h"
#include "ur_robot_driver/ur/tool_communication.h"
#include "ur_robot_driver/ur/version_information.h"
#include "ur_robot_driver/primary/robot_message/version_message.h"
#include "ur_robot_driver/rtde/rtde_writer.h"

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
   * \param script_file URScript file that should be sent to the robot.
   * \param output_recipe_file Filename where the output recipe is stored in.
   * \param input_recipe_file Filename where the input recipe is stored in.
   * \param handle_program_state Function handle to a callback on program state changes.
   * \param headless_mode Parameter to control if the driver should be started in headless mode.
   * \param tool_comm_setup Configuration for using the tool communication.
   * \param calibration_checksum Expected checksum of calibration. Will be matched against the
   * calibration reported by the robot.
   * \param reverse_port Port that will be opened by the driver to allow direct communication between the driver
   * and the robot controller.
   * \param script_sending_port The driver will offer an interface to receive the program's URScript on this port. If
   * the robot cannot connect to this port, `External Control` will stop immediately.
   * \param non_blocking_read Enable non-blocking mode for read (useful when used with combined_robot_hw)
   * \param servoj_gain Proportional gain for arm joints following target position, range [100,2000]
   * \param servoj_lookahead_time Time [S], range [0.03,0.2] smoothens the trajectory with this lookahead time
   */
  UrDriver(const std::string& robot_ip, const std::string& script_file, const std::string& output_recipe_file,
           const std::string& input_recipe_file, std::function<void(bool)> handle_program_state, bool headless_mode,
           std::unique_ptr<ToolCommSetup> tool_comm_setup, const std::string& calibration_checksum = "",
           const uint32_t reverse_port = 50001, const uint32_t script_sender_port = 50002, int servoj_gain = 2000,
           double servoj_lookahead_time = 0.03, bool non_blocking_read = false);
  /*!
   * \brief Constructs a new UrDriver object.
   *
   * \param robot_ip IP-address under which the robot is reachable.
   * \param script_file URScript file that should be sent to the robot.
   * \param output_recipe_file Filename where the output recipe is stored in.
   * \param input_recipe_file Filename where the input recipe is stored in.
   * \param handle_program_state Function handle to a callback on program state changes.
   * \param headless_mode Parameter to control if the driver should be started in headless mode.
   * \param calibration_checksum Expected checksum of calibration. Will be matched against the
   * calibration reported by the robot.
   * \param reverse_port Port that will be opened by the driver to allow direct communication between the driver
   * and the robot controller
   * \param script_sending_port The driver will offer an interface to receive the program's URScript on this port.
   * If the robot cannot connect to this port, `External Control` will stop immediately.
   * \param non_blocking_read Enable non-blocking mode for read (useful when used with combined_robot_hw)
   * \param servoj_gain Proportional gain for arm joints following target position, range [100,2000]
   * \param servoj_lookahead_time Time [S], range [0.03,0.2] smoothens the trajectory with this lookahead time
   */
  UrDriver(const std::string& robot_ip, const std::string& script_file, const std::string& output_recipe_file,
           const std::string& input_recipe_file, std::function<void(bool)> handle_program_state, bool headless_mode,
           const std::string& calibration_checksum = "", const uint32_t reverse_port = 50001,
           const uint32_t script_sender_port = 50002, int servoj_gain = 2000, double servoj_lookahead_time = 0.03,
           bool non_blocking_read = false)
    : UrDriver(robot_ip, script_file, output_recipe_file, input_recipe_file, handle_program_state, headless_mode,
               std::unique_ptr<ToolCommSetup>{}, calibration_checksum, reverse_port, script_sender_port, servoj_gain,
               servoj_lookahead_time, non_blocking_read)
  {
  }

  virtual ~UrDriver() = default;

  /*!
   * \brief Access function to receive the latest data package sent from the robot through RTDE
   * interface.
   *
   * \returns The latest data package on success, a nullptr if no package can be found inside a preconfigured time
   * window.
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
   * \param control_mode Control mode this command is assigned to.
   *
   * \returns True on successful write.
   */
  bool writeJointCommand(const vector6d_t& values, const comm::ControlMode control_mode);

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
   * \brief Starts the RTDE communication.
   *
   * After initialization, the cyclic RTDE communication is not started automatically, so that data
   * consumers can be started also at a later point.
   */
  void startRTDECommunication();

  /*!
   * \brief Sends a stop command to the socket interface which will signal the program running on
   * the robot to no longer listen for commands sent from the remote pc.
   *
   * \returns True on successful write.
   */
  bool stopControl();

  /*!
   * \brief Starts the watchdog checking if the URCaps program is running on the robot and it is
   * ready to receive control commands.
   */
  void startWatchdog();

  /*!
   * \brief Checks if the kinematics information in the used model fits the actual robot.
   *
   * \param checksum Hash of the used kinematics information
   */
  void checkCalibration(const std::string& checksum);

  /*!
   * \brief Getter for the RTDE writer used to write to the robot's RTDE interface.
   *
   * \returns The active RTDE writer
   */
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

  /*!
   * \brief Sends the external control program to the robot.
   *
   * Only for use in headless mode, as it replaces the use of the URCaps program.
   *
   * \returns true on successful upload, false otherwise
   */
  bool sendRobotProgram();

  /*!
   * \brief Returns version information about the currently connected robot
   */
  const VersionInformation& getVersion()
  {
    return robot_version_;
  }

private:
  std::string readScriptFile(const std::string& filename);
  std::string readKeepalive();

  int rtde_frequency_;
  comm::INotifier notifier_;
  std::unique_ptr<rtde_interface::RTDEClient> rtde_client_;
  std::unique_ptr<comm::ReverseInterface> reverse_interface_;
  std::unique_ptr<comm::ScriptSender> script_sender_;
  std::unique_ptr<comm::URStream<primary_interface::PrimaryPackage>> primary_stream_;
  std::unique_ptr<comm::URStream<primary_interface::PrimaryPackage>> secondary_stream_;

  double servoj_time_;
  uint32_t servoj_gain_;
  double servoj_lookahead_time_;

  std::thread watchdog_thread_;
  bool reverse_interface_active_;
  uint32_t reverse_port_;
  std::function<void(bool)> handle_program_state_;

  std::string robot_ip_;
  bool in_headless_mode_;
  std::string full_robot_program_;

  int get_packet_timeout_;
  bool non_blocking_read_;

  VersionInformation robot_version_;
};
}  // namespace ur_driver
#endif  // ifndef UR_RTDE_DRIVER_UR_UR_DRIVER_H_INCLUDED
