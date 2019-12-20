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
#ifndef UR_DRIVER_GEOMETRY_DATA_PUBLISHERS_H_INCLUDED
#define UR_DRIVER_GEOMETRY_DATA_PUBLISHERS_H_INCLUDED

#include <realtime_tools/realtime_publisher.h>
#include <ur_robot_driver/rtde/data_package.h>
#include <ur_robot_driver/ros/data_field_publisher.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace ur_driver
{
namespace rtde_interface
{
/*!
 * \brief Implements a publisher that publishes a datafield containing a pose.
 *
 */
class PosePublisher : public DataFieldPublisher
{
public:
  /*!
   * \brief Creates a PosePublisher object.
   *
   * \param data_field_identifier The string identifier of the data field to publish
   * \param nh The used ROS node handle
   */
  PosePublisher(const std::string& data_field_identifier, ros::NodeHandle& nh)
    : data_field_identifier_(data_field_identifier), pub_(nh, "rtde_data/" + data_field_identifier_, 1)
  {
    pub_.msg_ = geometry_msgs::Pose();
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
        tcp_angle_ = std::sqrt(std::pow(data_[3], 2) + std::pow(data_[4], 2) + std::pow(data_[5], 2));

        rotation_vec_ = tf2::Vector3(data_[3], data_[4], data_[5]);
        if (tcp_angle_ > 1e-16)
        {
          rotation_.setRotation(rotation_vec_.normalized(), tcp_angle_);
        }
        pub_.msg_.position.x = data_[0];
        pub_.msg_.position.y = data_[1];
        pub_.msg_.position.z = data_[2];

        pub_.msg_.orientation = tf2::toMsg(rotation_);

        pub_.unlockAndPublish();
        return true;
      }
    }
    return false;
  }

private:
  std::array<double, 6> data_;
  std::string data_field_identifier_;
  realtime_tools::RealtimePublisher<geometry_msgs::Pose> pub_;

  double tcp_angle_;
  tf2::Vector3 rotation_vec_;
  tf2::Quaternion rotation_;
};

/*!
 * \brief Implements a publisher that publishes a datafield containing a velocity.
 *
 */
class TwistPublisher : public DataFieldPublisher
{
public:
  /*!
   * \brief Creates a TwistPublisher object.
   *
   * \param data_field_identifier The string identifier of the data field to publish
   * \param nh The used ROS node handle
   */
  TwistPublisher(const std::string& data_field_identifier, ros::NodeHandle& nh)
    : data_field_identifier_(data_field_identifier), pub_(nh, "rtde_data/" + data_field_identifier_, 1)
  {
    pub_.msg_ = geometry_msgs::Twist();
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
        pub_.msg_.linear.x = data_[0];
        pub_.msg_.linear.y = data_[1];
        pub_.msg_.linear.z = data_[2];
        pub_.msg_.angular.x = data_[3];
        pub_.msg_.angular.y = data_[4];
        pub_.msg_.angular.z = data_[5];
        pub_.unlockAndPublish();
        return true;
      }
    }
    return false;
  }

private:
  std::array<double, 6> data_;
  std::string data_field_identifier_;
  realtime_tools::RealtimePublisher<geometry_msgs::Twist> pub_;
};

/*!
 * \brief Implements a publisher that publishes a datafield containing a 3D vector.
 *
 */
class Vector3Publisher : public DataFieldPublisher
{
public:
  /*!
   * \brief Creates a Vector3Publisher object.
   *
   * \param data_field_identifier The string identifier of the data field to publish
   * \param nh The used ROS node handle
   */
  Vector3Publisher(const std::string& data_field_identifier, ros::NodeHandle& nh)
    : data_field_identifier_(data_field_identifier), pub_(nh, "rtde_data/" + data_field_identifier_, 1)
  {
    pub_.msg_ = geometry_msgs::Vector3();
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
        pub_.msg_.x = data_[0];
        pub_.msg_.y = data_[1];
        pub_.msg_.z = data_[2];
        pub_.unlockAndPublish();
        return true;
      }
    }
    return false;
  }

private:
  std::array<double, 3> data_;
  std::string data_field_identifier_;
  realtime_tools::RealtimePublisher<geometry_msgs::Vector3> pub_;
};
}  // namespace rtde_interface
}  // namespace ur_driver

#endif  // ifndef UR_DRIVER_GEOMETRY_DATA_PUBLISHERS_H_INCLUDED
