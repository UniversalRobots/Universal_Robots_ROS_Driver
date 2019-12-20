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
#ifndef UR_DRIVER_SENSOR_DATA_PUBLISHERS_H_INCLUDED
#define UR_DRIVER_SENSOR_DATA_PUBLISHERS_H_INCLUDED

#include <realtime_tools/realtime_publisher.h>
#include <ur_robot_driver/rtde/data_package.h>
#include <ur_robot_driver/ros/data_field_publisher.h>
#include <std_msgs/Duration.h>

namespace ur_driver
{
namespace rtde_interface
{
/*!
 * \brief Implements a publisher that publishes a datafield containing a duration.
 *
 */
class DurationPublisher : public DataFieldPublisher
{
public:
  /*!
   * \brief Creates a DurationPublisher object.
   *
   * \param data_field_identifier The string identifier of the data field to publish
   * \param nh The used ROS node handle
   */
  DurationPublisher(const std::string& data_field_identifier, ros::NodeHandle& nh)
    : data_field_identifier_(data_field_identifier), pub_(nh, "rtde_data/" + data_field_identifier_, 1)
  {
    pub_.msg_ = std_msgs::Duration();
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
        pub_.msg_.data.fromSec(data_);
        pub_.unlockAndPublish();
        return true;
      }
    }
    return false;
  }

private:
  double data_;
  std::string data_field_identifier_;
  realtime_tools::RealtimePublisher<std_msgs::Duration> pub_;
};

/*!
 * \brief Implements a publisher that publishes a datafield containing the joint temperatures.
 *
 */
class JointTemperaturePublisher : public DataFieldPublisher
{
public:
  /*!
   * \brief Creates a JointTemperaturePublisher object.
   *
   * \param data_field_identifier The string identifier of the data field to publish
   * \param nh The used ROS node handle
   */
  JointTemperaturePublisher(const std::string& data_field_identifier, ros::NodeHandle& nh)
    : data_field_identifier_(data_field_identifier), pub_(nh, "rtde_data/" + data_field_identifier_, 1)
  {
    pub_.msg_ = std_msgs::Duration();
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
        pub_.msg_.data.fromSec(data_);
        pub_.unlockAndPublish();
        return true;
      }
    }
    return false;
  }

private:
  double data_;
  std::string data_field_identifier_;
  realtime_tools::RealtimePublisher<std_msgs::Duration> pub_;
};
}  // namespace rtde_interface
}  // namespace ur_driver

#endif  // ifndef UR_DRIVER_SENSOR_DATA_PUBLISHERS_H_INCLUDED
