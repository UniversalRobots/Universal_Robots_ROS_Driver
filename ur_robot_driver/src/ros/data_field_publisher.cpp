#include "ur_robot_driver/ros/data_field_publisher.h"
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
 * \date    2019-10-30
 *
 */
//----------------------------------------------------------------------

#include "ur_robot_driver/exceptions.h"
#include <std_msgs/Bool.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/UInt32.h>
#include <std_msgs/UInt64.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Int32MultiArray.h>
#include <std_msgs/UInt32MultiArray.h>
#include <std_msgs/String.h>

namespace ur_driver
{
namespace rtde_interface
{
using _bool_publisher = DirectDataPublisher<bool, std_msgs::Bool>;
using _uint8_publisher = DirectDataPublisher<uint8_t, std_msgs::UInt8>;
using _uint32_publisher = DirectDataPublisher<uint32_t, std_msgs::UInt32>;
using _uint64_publisher = DirectDataPublisher<uint64_t, std_msgs::UInt64>;
using _int32_publisher = DirectDataPublisher<int32_t, std_msgs::Int32>;
using _double_publisher = DirectDataPublisher<double, std_msgs::Float64>;
using _vector3d_publisher = ArrayDataPublisher<double, std_msgs::Float64MultiArray, 3>;
using _vector6d_publisher = ArrayDataPublisher<double, std_msgs::Float64MultiArray, 6>;
using _vector6int32_publisher = ArrayDataPublisher<int32_t, std_msgs::Int32MultiArray, 6>;
using _vector6uint32_publisher = ArrayDataPublisher<uint32_t, std_msgs::UInt32MultiArray, 6>;
using _string_publisher = DirectDataPublisher<std::string, std_msgs::String>;

std::unique_ptr<DataFieldPublisher> DataFieldPublisher::createFromString(const std::string& data_field_identifier,
                                                                         ros::NodeHandle& nh)
{
  if (DataPackage::isType<bool>(data_field_identifier))
  {
    return std::unique_ptr<DataFieldPublisher>(new _bool_publisher(data_field_identifier, nh));
  }
  else if (DataPackage::isType<uint8_t>(data_field_identifier))
  {
    return std::unique_ptr<DataFieldPublisher>(new _uint8_publisher(data_field_identifier, nh));
  }
  else if (DataPackage::isType<uint32_t>(data_field_identifier))
  {
    return std::unique_ptr<DataFieldPublisher>(new _uint32_publisher(data_field_identifier, nh));
  }
  else if (DataPackage::isType<uint64_t>(data_field_identifier))
  {
    return std::unique_ptr<DataFieldPublisher>(new _uint64_publisher(data_field_identifier, nh));
  }
  else if (DataPackage::isType<int32_t>(data_field_identifier))
  {
    return std::unique_ptr<DataFieldPublisher>(new _int32_publisher(data_field_identifier, nh));
  }
  else if (DataPackage::isType<double_t>(data_field_identifier))
  {
    return std::unique_ptr<DataFieldPublisher>(new _double_publisher(data_field_identifier, nh));
  }
  else if (DataPackage::isType<vector3d_t>(data_field_identifier))
  {
    return std::unique_ptr<DataFieldPublisher>(new _vector3d_publisher(data_field_identifier, nh));
  }
  else if (DataPackage::isType<vector6d_t>(data_field_identifier))
  {
    return std::unique_ptr<DataFieldPublisher>(new _vector6d_publisher(data_field_identifier, nh));
  }
  else if (DataPackage::isType<vector6int32_t>(data_field_identifier))
  {
    return std::unique_ptr<DataFieldPublisher>(new _vector6int32_publisher(data_field_identifier, nh));
  }
  else if (DataPackage::isType<vector6uint32_t>(data_field_identifier))
  {
    return std::unique_ptr<DataFieldPublisher>(new _vector6uint32_publisher(data_field_identifier, nh));
  }
  else if (DataPackage::isType<std::string>(data_field_identifier))
  {
    return std::unique_ptr<DataFieldPublisher>(new _string_publisher(data_field_identifier, nh));
  }
  else
  {
    throw UrException("No supported publisher for RTDE data field " + data_field_identifier);
  }
}
}  // namespace rtde_interface
}  // namespace ur_driver
