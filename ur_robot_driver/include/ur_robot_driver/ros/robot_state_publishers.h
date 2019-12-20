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
 * \author  Tristan Schnell schnell@fzi.de
 * \date    2019-12-20
 *
 */
//----------------------------------------------------------------------
#ifndef UR_DRIVER_ROBOT_STATE_PUBLISHERS_H_INCLUDED
#define UR_DRIVER_ROBOT_STATE_PUBLISHERS_H_INCLUDED

#include <realtime_tools/realtime_publisher.h>
#include <ur_robot_driver/rtde/data_package.h>
#include <ur_robot_driver/ros/data_field_publisher.h>
#include <ur_rtde_msgs/JointMode.h>
#include <ur_rtde_msgs/RobotStatusBits.h>
#include <ur_rtde_msgs/SafetyStatusBits.h>

namespace ur_driver
{
namespace rtde_interface
{
/*!
 * \brief Implements a publisher that directly publishes a datafield containing an arbitrary mode.
 *
 */
template <typename DataT, typename MsgT>
class ModePublisher : public DataFieldPublisher
{
public:
  /*!
   * \brief Creates a ModePublisher object.
   *
   * \param data_field_identifier The string identifier of the data field to publish
   * \param nh The used ROS node handle
   */
  ModePublisher(const std::string& data_field_identifier, ros::NodeHandle& nh)
    : data_field_identifier_(data_field_identifier), pub_(nh, "rtde_data/" + data_field_identifier_, 1)
  {
  }

  /*!
   * \brief Publishes the relevant data field from a data package.
   *
   * \param data_package The given data package to publish from
   *
   * \returns True if the realtime publisher could publish the data.
   */
  virtual bool publish(const DataPackage& data_package)
  {
    if (data_package.getData(data_field_identifier_, data_))
    {
      if (pub_.trylock())
      {
        pub_.msg_.mode = data_;
        pub_.unlockAndPublish();
        return true;
      }
    }
    return false;
  }

private:
  DataT data_;
  std::string data_field_identifier_;
  realtime_tools::RealtimePublisher<MsgT> pub_;
};

/*!
 * \brief Implements a publisher that directly publishes a datafield containing an arbitrary status.
 *
 */
template <typename DataT, typename MsgT>
class StatusPublisher : public DataFieldPublisher
{
public:
  /*!
   * \brief Creates a StatusPublisher object.
   *
   * \param data_field_identifier The string identifier of the data field to publish
   * \param nh The used ROS node handle
   */
  StatusPublisher(const std::string& data_field_identifier, ros::NodeHandle& nh)
    : data_field_identifier_(data_field_identifier), pub_(nh, "rtde_data/" + data_field_identifier_, 1)
  {
  }

  /*!
   * \brief Publishes the relevant data field from a data package.
   *
   * \param data_package The given data package to publish from
   *
   * \returns True if the realtime publisher could publish the data.
   */
  virtual bool publish(const DataPackage& data_package)
  {
    if (data_package.getData(data_field_identifier_, data_))
    {
      if (pub_.trylock())
      {
        pub_.msg_.status = data_;
        pub_.unlockAndPublish();
        return true;
      }
    }
    return false;
  }

private:
  DataT data_;
  std::string data_field_identifier_;
  realtime_tools::RealtimePublisher<MsgT> pub_;
};

/*!
 * \brief Implements a publisher that publishes a datafield containing a the joint modes.
 *
 */
class JointModePublisher : public DataFieldPublisher
{
public:
  /*!
   * \brief Creates a JointModePublisher object.
   *
   * \param data_field_identifier The string identifier of the data field to publish
   * \param nh The used ROS node handle
   */
  JointModePublisher(const std::string& data_field_identifier, ros::NodeHandle& nh)
    : data_field_identifier_(data_field_identifier), pub_(nh, "rtde_data/" + data_field_identifier_, 1)
  {
    pub_.msg_ = ur_rtde_msgs::JointMode();
    pub_.msg_.mode.resize(NUM_JOINTS);
  }

  /*!
   * \brief Publishes the relevant data field from a data package.
   *
   * \param data_package The given data package to publish from
   *
   * \returns True if the realtime publisher could publish the data.
   */
  virtual bool publish(const DataPackage& data_package)
  {
    if (data_package.getData(data_field_identifier_, data_))
    {
      if (pub_.trylock())
      {
        for (size_t i = 0; i < NUM_JOINTS; i++)
        {
          pub_.msg_.mode[i] = data_[i];
        }
        pub_.unlockAndPublish();
        return true;
      }
    }
    return false;
  }

private:
  static const size_t NUM_JOINTS = 6;
  std::array<int32_t, NUM_JOINTS> data_;
  std::string data_field_identifier_;
  realtime_tools::RealtimePublisher<ur_rtde_msgs::JointMode> pub_;
};

/*!
 * \brief Implements a publisher that publishes a datafield containing a the robot status bits.
 *
 */
class RobotStatusBitsPublisher : public DataFieldPublisher
{
public:
  /*!
   * \brief Creates a RobotStatusBitsPublisher object.
   *
   * \param data_field_identifier The string identifier of the data field to publish
   * \param nh The used ROS node handle
   */
  RobotStatusBitsPublisher(const std::string& data_field_identifier, ros::NodeHandle& nh)
    : data_field_identifier_(data_field_identifier), pub_(nh, "rtde_data/" + data_field_identifier_, 1)
  {
    pub_.msg_ = ur_rtde_msgs::RobotStatusBits();
  }

  /*!
   * \brief Publishes the relevant data field from a data package.
   *
   * \param data_package The given data package to publish from
   *
   * \returns True if the realtime publisher could publish the data.
   */
  virtual bool publish(const DataPackage& data_package)
  {
    if (data_package.getData<uint32_t, ARRAY_SIZE>(data_field_identifier_, data_))
    {
      if (pub_.trylock())
      {
        pub_.msg_.is_power_on = data_[0];
        pub_.msg_.is_program_running = data_[1];
        pub_.msg_.is_teach_button_pressed = data_[2];
        pub_.msg_.is_power_button_pressed = data_[3];
        pub_.unlockAndPublish();
        return true;
      }
    }
    return false;
  }

private:
  static const size_t ARRAY_SIZE = 32;
  std::bitset<ARRAY_SIZE> data_;
  std::string data_field_identifier_;
  realtime_tools::RealtimePublisher<ur_rtde_msgs::RobotStatusBits> pub_;
};

/*!
 * \brief Implements a publisher that publishes a datafield containing a the safety status bits.
 *
 */
class SafetyStatusBitsPublisher : public DataFieldPublisher
{
public:
  /*!
   * \brief Creates a SafetyStatusBitsPublisher object.
   *
   * \param data_field_identifier The string identifier of the data field to publish
   * \param nh The used ROS node handle
   */
  SafetyStatusBitsPublisher(const std::string& data_field_identifier, ros::NodeHandle& nh)
    : data_field_identifier_(data_field_identifier), pub_(nh, "rtde_data/" + data_field_identifier_, 1)
  {
    pub_.msg_ = ur_rtde_msgs::SafetyStatusBits();
  }

  /*!
   * \brief Publishes the relevant data field from a data package.
   *
   * \param data_package The given data package to publish from
   *
   * \returns True if the realtime publisher could publish the data.
   */
  virtual bool publish(const DataPackage& data_package)
  {
    if (data_package.getData<uint32_t, ARRAY_SIZE>(data_field_identifier_, data_))
    {
      if (pub_.trylock())
      {
        pub_.msg_.is_normal_mode = data_[0];
        pub_.msg_.is_reduced_mode = data_[1];
        pub_.msg_.is_protective_stopped = data_[2];
        pub_.msg_.is_recovery_mode = data_[3];
        pub_.msg_.is_safeguard_stopped = data_[4];
        pub_.msg_.is_system_emergency_stopped = data_[5];
        pub_.msg_.is_robot_emergency_stopped = data_[6];
        pub_.msg_.is_emergency_stopped = data_[7];
        pub_.msg_.is_violation = data_[8];
        pub_.msg_.is_fault = data_[9];
        pub_.msg_.is_stopped_due_to_safety = data_[10];
        pub_.unlockAndPublish();
        return true;
      }
    }
    return false;
  }

private:
  static const size_t ARRAY_SIZE = 32;
  std::bitset<ARRAY_SIZE> data_;
  std::string data_field_identifier_;
  realtime_tools::RealtimePublisher<ur_rtde_msgs::SafetyStatusBits> pub_;
};
}  // namespace rtde_interface
}  // namespace ur_driver

#endif  // ifndef UR_DRIVER_ROBOT_STATE_PUBLISHERS_H_INCLUDED
