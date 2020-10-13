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
#ifndef UR_DRIVER_HARDWARE_INTERFACE_H_INCLUDED
#define UR_DRIVER_HARDWARE_INTERFACE_H_INCLUDED

#include <hardware_interface/robot_hw.h>
#include <hardware_interface/force_torque_sensor_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <algorithm>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>
#include <std_srvs/Trigger.h>
#include <realtime_tools/realtime_publisher.h>
#include "tf2_msgs/TFMessage.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <ur_msgs/IOStates.h>
#include <ur_msgs/ToolDataMsg.h>
#include <ur_msgs/SetIO.h>
#include "ur_msgs/SetSpeedSliderFraction.h"

#include <ur_controllers/speed_scaling_interface.h>
#include <ur_controllers/scaled_joint_command_interface.h>

#include "ur_robot_driver/ur/ur_driver.h"
#include <ur_robot_driver/ros/dashboard_client_ros.h>

#include <ur_dashboard_msgs/RobotMode.h>
#include <ur_dashboard_msgs/SafetyMode.h>

#include <industrial_robot_status_interface/industrial_robot_status_interface.h>

namespace ur_driver
{
/*!
 * \brief Possible states for robot control
 */
enum class PausingState
{
  PAUSED,
  RUNNING,
  RAMPUP
};

/*!
 * \brief The HardwareInterface class handles the interface between the ROS system and the main
 * driver. It contains the read and write methods of the main control loop and registers various ROS
 * topics and services.
 */
class HardwareInterface : public hardware_interface::RobotHW
{
public:
  /*!
   * \brief Creates a new HardwareInterface object.
   */
  HardwareInterface();
  virtual ~HardwareInterface() = default;
  /*!
   * \brief Handles the setup functionality for the ROS interface. This includes parsing ROS
   * parameters, creating interfaces, starting the main driver and advertising ROS services.
   *
   * \param root_nh Root level ROS node handle
   * \param robot_hw_nh ROS node handle for the robot namespace
   *
   * \returns True, if the setup was performed successfully
   */
  virtual bool init(ros::NodeHandle& root_nh, ros::NodeHandle& robot_hw_nh) override;
  /*!
   * \brief Read method of the control loop. Reads a RTDE package from the robot and handles and
   * publishes the information as needed.
   *
   * \param time Current time
   * \param period Duration of current control loop iteration
   */
  virtual void read(const ros::Time& time, const ros::Duration& period) override;
  /*!
   * \brief Write method of the control loop. Writes target joint positions to the robot to be read
   * by its URCaps program.
   *
   * \param time Current time
   * \param period Duration of current control loop iteration
   */
  virtual void write(const ros::Time& time, const ros::Duration& period) override;
  /*!
   * \brief Preparation to start and stop loaded controllers.
   *
   * \param start_list List of controllers to start
   * \param stop_list List of controllers to stop
   *
   * \returns True, if the controllers can be switched
   */
  virtual bool prepareSwitch(const std::list<hardware_interface::ControllerInfo>& start_list,
                             const std::list<hardware_interface::ControllerInfo>& stop_list) override;
  /*!
   * \brief Starts and stops controllers.
   *
   * \param start_list List of controllers to start
   * \param stop_list List of controllers to stop
   */
  virtual void doSwitch(const std::list<hardware_interface::ControllerInfo>& start_list,
                        const std::list<hardware_interface::ControllerInfo>& stop_list) override;

  /*!
   * \brief Getter for the current control frequency
   *
   * \returns The used control frequency
   */
  uint32_t getControlFrequency() const;

  /*!
   * \brief Checks if the URCaps program is running on the robot.
   *
   * \returns True, if the program is currently running, false otherwise.
   */
  bool isRobotProgramRunning() const;

  /*!
   * \brief Callback to handle a change in the current state of the URCaps program running on the
   * robot.
   *
   * \param program_running The new state of the program
   */
  void handleRobotProgramState(bool program_running);

  /*!
   * \brief Checks if a reset of the ROS controllers is necessary.
   *
   * \returns Necessity of ROS controller reset
   */
  bool shouldResetControllers();

protected:
  /*!
   * \brief Transforms force-torque measurements reported from the robot from base to tool frame.
   *
   * Requires extractToolPose() to be run first.
   */
  void transformForceTorque();

  /*!
   * \brief Stores the raw tool pose data from the robot in a transformation msg
   *
   * \param timestamp Timestamp of read data
   */
  void extractToolPose(const ros::Time& timestamp);

  /*!
   * \brief Publishes the tool pose to the tf system
   *
   * Requires extractToolPose() to be run first.
   */
  void publishPose();

  void publishIOData();
  void publishToolData();
  void publishRobotAndSafetyMode();

  /*!
   * \brief Read and evaluate data in order to set robot status properties for industrial
   *        robot status interface
   */
  void extractRobotStatus();

  bool stopControl(std_srvs::TriggerRequest& req, std_srvs::TriggerResponse& res);

  template <typename T>
  void readData(const std::unique_ptr<rtde_interface::DataPackage>& data_pkg, const std::string& var_name, T& data);
  template <typename T, size_t N>
  void readBitsetData(const std::unique_ptr<rtde_interface::DataPackage>& data_pkg, const std::string& var_name,
                      std::bitset<N>& data);

  bool setSpeedSlider(ur_msgs::SetSpeedSliderFractionRequest& req, ur_msgs::SetSpeedSliderFractionResponse& res);
  bool setIO(ur_msgs::SetIORequest& req, ur_msgs::SetIOResponse& res);
  bool resendRobotProgram(std_srvs::TriggerRequest& req, std_srvs::TriggerResponse& res);
  bool zeroFTSensor(std_srvs::TriggerRequest& req, std_srvs::TriggerResponse& res);
  void commandCallback(const std_msgs::StringConstPtr& msg);

  std::unique_ptr<UrDriver> ur_driver_;
  std::unique_ptr<DashboardClientROS> dashboard_client_;

  /*!
   * \brief Checks whether a resource list contains joints from this hardware interface
   *
   * True is returned as soon as one joint name from claimed_resources matches a joint from this
   * hardware interface.
   */
  bool checkControllerClaims(const std::set<std::string>& claimed_resources);

  ros::ServiceServer deactivate_srv_;
  ros::ServiceServer tare_sensor_srv_;
  ros::ServiceServer set_payload_srv_;

  hardware_interface::JointStateInterface js_interface_;
  ur_controllers::ScaledPositionJointInterface spj_interface_;
  hardware_interface::PositionJointInterface pj_interface_;
  ur_controllers::SpeedScalingInterface speedsc_interface_;
  hardware_interface::VelocityJointInterface vj_interface_;
  ur_controllers::ScaledVelocityJointInterface svj_interface_;
  hardware_interface::ForceTorqueSensorInterface fts_interface_;

  vector6d_t joint_position_command_;
  vector6d_t joint_velocity_command_;
  vector6d_t joint_positions_;
  vector6d_t joint_velocities_;
  vector6d_t joint_efforts_;
  vector6d_t fts_measurements_;
  vector6d_t tcp_pose_;
  std::bitset<18> actual_dig_out_bits_;
  std::bitset<18> actual_dig_in_bits_;
  std::array<double, 2> standard_analog_input_;
  std::array<double, 2> standard_analog_output_;
  std::bitset<4> analog_io_types_;
  uint32_t tool_mode_;
  std::bitset<2> tool_analog_input_types_;
  std::array<double, 2> tool_analog_input_;
  int32_t tool_output_voltage_;
  double tool_output_current_;
  double tool_temperature_;
  tf2::Vector3 tcp_force_;
  tf2::Vector3 tcp_torque_;
  geometry_msgs::TransformStamped tcp_transform_;
  double speed_scaling_;
  double target_speed_fraction_;
  double speed_scaling_combined_;
  std::vector<std::string> joint_names_;
  int32_t robot_mode_;
  int32_t safety_mode_;
  std::bitset<4> robot_status_bits_;
  std::bitset<11> safety_status_bits_;

  std::unique_ptr<realtime_tools::RealtimePublisher<tf2_msgs::TFMessage>> tcp_pose_pub_;
  std::unique_ptr<realtime_tools::RealtimePublisher<ur_msgs::IOStates>> io_pub_;
  std::unique_ptr<realtime_tools::RealtimePublisher<ur_msgs::ToolDataMsg>> tool_data_pub_;
  std::unique_ptr<realtime_tools::RealtimePublisher<ur_dashboard_msgs::RobotMode>> robot_mode_pub_;
  std::unique_ptr<realtime_tools::RealtimePublisher<ur_dashboard_msgs::SafetyMode>> safety_mode_pub_;

  ros::ServiceServer set_speed_slider_srv_;
  ros::ServiceServer set_io_srv_;
  ros::ServiceServer resend_robot_program_srv_;
  ros::Subscriber command_sub_;

  industrial_robot_status_interface::RobotStatus robot_status_resource_{};
  industrial_robot_status_interface::IndustrialRobotStatusInterface robot_status_interface_{};

  uint32_t runtime_state_;
  bool position_controller_running_;
  bool velocity_controller_running_;

  PausingState pausing_state_;
  double pausing_ramp_up_increment_;

  std::string tcp_link_;
  bool robot_program_running_;
  ros::Publisher program_state_pub_;

  bool controller_reset_necessary_;
  bool controllers_initialized_;

  bool packet_read_;
  bool non_blocking_read_;

  std::string robot_ip_;
  std::string tf_prefix_;
};

}  // namespace ur_driver

#endif  // ifndef UR_DRIVER_HARDWARE_INTERFACE_H_INCLUDED
