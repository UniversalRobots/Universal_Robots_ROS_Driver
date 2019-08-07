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

#include <ur_controllers/speed_scaling_interface.h>
#include <ur_controllers/scaled_joint_command_interface.h>

#include "ur_rtde_driver/ur/ur_driver.h"
#include "ur_rtde_msgs/SetSpeedSlider.h"

namespace ur_driver
{
enum class PausingState
{
  PAUSED,
  RUNNING,
  RAMPUP
};

class HardwareInterface : public hardware_interface::RobotHW
{
public:
  HardwareInterface();
  virtual ~HardwareInterface() = default;
  virtual bool init(ros::NodeHandle& root_nh, ros::NodeHandle& robot_hw_nh) override;
  virtual void read(const ros::Time& time, const ros::Duration& period) override;
  virtual void write(const ros::Time& time, const ros::Duration& period) override;
  virtual bool prepareSwitch(const std::list<hardware_interface::ControllerInfo>& start_list,
                             const std::list<hardware_interface::ControllerInfo>& stop_list) override;
  virtual void doSwitch(const std::list<hardware_interface::ControllerInfo>& start_list,
                        const std::list<hardware_interface::ControllerInfo>& stop_list) override;

  uint32_t getControlFrequency() const;

  bool isRobotProgramRunning() const;

  void handleRobotProgramState(bool program_running);

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

  bool stopControl(std_srvs::TriggerRequest& req, std_srvs::TriggerResponse& res);

  template <typename T>
  void readData(const std::unique_ptr<rtde_interface::DataPackage>& data_pkg, const std::string& var_name, T& data);
  template <typename T, size_t N>
  void readBitsetData(const std::unique_ptr<rtde_interface::DataPackage>& data_pkg, const std::string& var_name,
                      std::bitset<N>& data);

  bool setSpeedSlider(ur_rtde_msgs::SetSpeedSliderRequest& req, ur_rtde_msgs::SetSpeedSliderResponse& res);
  bool setIO(ur_msgs::SetIORequest& req, ur_msgs::SetIOResponse& res);
  void commandCallback(const std_msgs::StringConstPtr& msg);

  std::unique_ptr<UrDriver> ur_driver_;

  ros::ServiceServer deactivate_srv_;

  hardware_interface::JointStateInterface js_interface_;
  ur_controllers::ScaledPositionJointInterface spj_interface_;
  hardware_interface::PositionJointInterface pj_interface_;
  ur_controllers::SpeedScalingInterface speedsc_interface_;
  // hardware_interface::VelocityJointInterface vj_interface_;
  hardware_interface::ForceTorqueSensorInterface fts_interface_;

  vector6d_t joint_position_command_;
  // std::vector<double> joint_velocity_command_;
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

  std::unique_ptr<realtime_tools::RealtimePublisher<tf2_msgs::TFMessage>> tcp_pose_pub_;
  std::unique_ptr<realtime_tools::RealtimePublisher<ur_msgs::IOStates>> io_pub_;
  std::unique_ptr<realtime_tools::RealtimePublisher<ur_msgs::ToolDataMsg>> tool_data_pub_;

  ros::ServiceServer set_speed_slider_srv_;
  ros::ServiceServer set_io_srv_;
  ros::Subscriber command_sub_;

  uint32_t runtime_state_;
  bool position_controller_running_;

  PausingState pausing_state_;
  double pausing_ramp_up_increment_;

  std::string tcp_link_;
  bool robot_program_running_;
  ros::Publisher program_state_pub_;

  bool controller_reset_necessary_;
  bool controllers_initialized_;

  std::string robot_ip_;
};

}  // namespace ur_driver

#endif  // ifndef UR_DRIVER_HARDWARE_INTERFACE_H_INCLUDED
