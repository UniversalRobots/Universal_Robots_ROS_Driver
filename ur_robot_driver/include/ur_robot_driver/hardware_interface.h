// this is for emacs file handling -*- mode: c++; indent-tabs-mode: nil -*-

// -- BEGIN LICENSE BLOCK ----------------------------------------------
// Copyright 2019 FZI Forschungszentrum Informatik
// Created on behalf of Universal Robots A/S
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
#include <pass_through_controllers/trajectory_interface.h>
#include <algorithm>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>
#include <std_srvs/Trigger.h>
#include <realtime_tools/realtime_publisher.h>
#include <tf2_msgs/TFMessage.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <control_msgs/FollowJointTrajectoryAction.h>
#include <control_msgs/FollowJointTrajectoryFeedback.h>
#include <cartesian_control_msgs/FollowCartesianTrajectoryAction.h>
#include <cartesian_control_msgs/FollowCartesianTrajectoryFeedback.h>

#include <ur_client_library/control/trajectory_point_interface.h>
#include <ur_msgs/IOStates.h>
#include <ur_msgs/ToolDataMsg.h>
#include <ur_msgs/SetIO.h>
#include <ur_msgs/SetSpeedSliderFraction.h>
#include <ur_msgs/SetPayload.h>

#include <cartesian_interface/cartesian_command_interface.h>
#include <cartesian_interface/cartesian_state_handle.h>

#include <speed_scaling_interface/speed_scaling_interface.h>
#include <scaled_joint_trajectory_controller/scaled_joint_command_interface.h>

#include <ur_client_library/ur/ur_driver.h>
#include <ur_robot_driver/dashboard_client_ros.h>

#include <ur_dashboard_msgs/RobotMode.h>
#include <ur_dashboard_msgs/SafetyMode.h>

#include <industrial_robot_status_interface/industrial_robot_status_interface.h>
#include <kdl/frames.hpp>

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
  void readData(const std::unique_ptr<urcl::rtde_interface::DataPackage>& data_pkg, const std::string& var_name,
                T& data);
  template <typename T, size_t N>
  void readBitsetData(const std::unique_ptr<urcl::rtde_interface::DataPackage>& data_pkg, const std::string& var_name,
                      std::bitset<N>& data);

  bool setSpeedSlider(ur_msgs::SetSpeedSliderFractionRequest& req, ur_msgs::SetSpeedSliderFractionResponse& res);
  bool setIO(ur_msgs::SetIORequest& req, ur_msgs::SetIOResponse& res);
  bool resendRobotProgram(std_srvs::TriggerRequest& req, std_srvs::TriggerResponse& res);
  bool zeroFTSensor(std_srvs::TriggerRequest& req, std_srvs::TriggerResponse& res);
  void commandCallback(const std_msgs::StringConstPtr& msg);
  bool setPayload(ur_msgs::SetPayloadRequest& req, ur_msgs::SetPayloadResponse& res);

  std::unique_ptr<urcl::UrDriver> ur_driver_;
  std::unique_ptr<DashboardClientROS> dashboard_client_;

  /*!
   * \brief Checks whether a resource list contains joints from this hardware interface
   *
   * True is returned as soon as one joint name from claimed_resources matches a joint from this
   * hardware interface.
   */
  bool checkControllerClaims(const std::set<std::string>& claimed_resources);

  void startJointInterpolation(const hardware_interface::JointTrajectory& trajectory);

  void startCartesianInterpolation(const hardware_interface::CartesianTrajectory& trajectory);

  void cancelInterpolation();

  void passthroughTrajectoryDoneCb(urcl::control::TrajectoryResult result);

  ros::ServiceServer deactivate_srv_;
  ros::ServiceServer tare_sensor_srv_;
  ros::ServiceServer set_payload_srv_;

  hardware_interface::JointStateInterface js_interface_;
  scaled_controllers::ScaledPositionJointInterface spj_interface_;
  hardware_interface::PositionJointInterface pj_interface_;
  scaled_controllers::SpeedScalingInterface speedsc_interface_;
  hardware_interface::VelocityJointInterface vj_interface_;
  scaled_controllers::ScaledVelocityJointInterface svj_interface_;
  hardware_interface::ForceTorqueSensorInterface fts_interface_;
  hardware_interface::JointTrajectoryInterface jnt_traj_interface_;
  hardware_interface::CartesianTrajectoryInterface cart_traj_interface_;

  ros_controllers_cartesian::CartesianStateInterface cart_interface_;
  ros_controllers_cartesian::TwistCommandInterface twist_interface_;
  ros_controllers_cartesian::PoseCommandInterface pose_interface_;

  geometry_msgs::Pose cart_pose_;
  geometry_msgs::Twist cart_twist_;
  geometry_msgs::Accel cart_accel_;
  geometry_msgs::Accel cart_jerk_;
  geometry_msgs::Twist twist_command_;
  geometry_msgs::Pose pose_command_;
  geometry_msgs::Pose target_cart_pose_;
  geometry_msgs::Twist target_cart_twist_;
  geometry_msgs::Pose error_cart_pose_;
  geometry_msgs::Twist error_cart_twist_;

  KDL::Vector tcp_vec_;
  double tcp_angle_;
  KDL::Rotation tcp_pose_rot_;
  KDL::Rotation target_tcp_pose_rot_;

  urcl::vector6d_t joint_position_command_;
  urcl::vector6d_t joint_velocity_command_;
  urcl::vector6d_t joint_positions_;
  urcl::vector6d_t joint_velocities_;
  urcl::vector6d_t target_joint_positions_;
  urcl::vector6d_t target_joint_velocities_;
  urcl::vector6d_t joint_efforts_;
  urcl::vector6d_t fts_measurements_;
  urcl::vector6d_t tcp_pose_;
  urcl::vector6d_t tcp_speed_;
  urcl::vector6d_t target_tcp_pose_;
  urcl::vector6d_t target_tcp_speed_;
  urcl::vector6d_t cartesian_velocity_command_;
  urcl::vector6d_t cartesian_pose_command_;
  urcl::vector6d_t tcp_offset_;

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
  std::atomic<bool> position_controller_running_;
  std::atomic<bool> velocity_controller_running_;
  std::atomic<bool> joint_forward_controller_running_;
  std::atomic<bool> cartesian_forward_controller_running_;
  std::atomic<bool> twist_controller_running_;
  std::atomic<bool> pose_controller_running_;

  PausingState pausing_state_;
  double pausing_ramp_up_increment_;

  std::string tcp_link_;
  std::atomic<bool> robot_program_running_;
  ros::Publisher program_state_pub_;

  std::atomic<bool> controller_reset_necessary_;
  bool controllers_initialized_;

  bool packet_read_;
  bool non_blocking_read_;

  std::string robot_ip_;
  std::string tf_prefix_;
};

}  // namespace ur_driver

#endif  // ifndef UR_DRIVER_HARDWARE_INTERFACE_H_INCLUDED
