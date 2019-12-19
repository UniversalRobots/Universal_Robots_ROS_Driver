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
#ifndef UR_DRIVER_DATA_FIELD_PUBLISHER_H_INCLUDED
#define UR_DRIVER_DATA_FIELD_PUBLISHER_H_INCLUDED

#include <realtime_tools/realtime_publisher.h>
#include <ur_robot_driver/rtde/data_package.h>

namespace ur_driver
{
namespace rtde_interface
{
/*!
 * \brief The DataFieldPublisher class implements an abstract wrapper for various ROS publishers
 * that publish fields of the RTDE data package. In addition, it contains a static factory to
 * create correct publishers for a data field.
 */
class DataFieldPublisher
{
public:
  DataFieldPublisher() = default;

  /*!
   * \brief Publishes partial information from a data package.
   *
   * \param data_package The given data package to publish from
   *
   * \returns True if the realtime publisher could publish the data.
   */
  virtual bool publish(const DataPackage& data_package) = 0;

  /*!
   * \brief Creates a DataFieldPublisher object based on a given data field.
   *
   * \param data_field_identifier The name of the data field to publish
   * \param nh The used ROS node handle
   *
   * \returns A unique pointer to the created Publisher object
   */
  static std::unique_ptr<DataFieldPublisher> createFromString(const std::string& data_field_identifier,
                                                              ros::NodeHandle& nh);
};

/*!
 * \brief Implements a publisher that directly publishes a datafield of a given type to a ROS topic
 * of a given message type.
 *
 */
template <typename DataT, typename MsgT>
class DirectDataPublisher : public DataFieldPublisher
{
public:
  /*!
   * \brief Creates a DirectDataPublisher object.
   *
   * \param data_field_identifier The string identifier of the data field to publish
   * \param nh The used ROS node handle
   */
  DirectDataPublisher(const std::string& data_field_identifier, ros::NodeHandle& nh)
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
        pub_.msg_.data = data_;
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
 * \brief Implements a publisher that publishes a datafield containing an array of a given type and size to a ROS topic
 * of a given message type.
 *
 */
template <typename DataT, typename MsgT, size_t N>
class ArrayDataPublisher : public DataFieldPublisher
{
public:
  /*!
   * \brief Creates a DirectDataPublisher object.
   *
   * \param data_field_identifier The string identifier of the data field to publish
   * \param nh The used ROS node handle
   */
  ArrayDataPublisher(const std::string& data_field_identifier, ros::NodeHandle& nh)
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
        for (size_t i = 0; i < N; i++)
        {
          pub_.msg_.data[i] = data_[i];
        }
        pub_.unlockAndPublish();
        return true;
      }
    }
    return false;
  }

private:
  std::array<DataT, N> data_;
  std::string data_field_identifier_;
  realtime_tools::RealtimePublisher<MsgT> pub_;
};
}  // namespace rtde_interface
}  // namespace ur_driver

#endif  // ifndef UR_DRIVER_DATA_FIELD_PUBLISHER_H_INCLUDED
