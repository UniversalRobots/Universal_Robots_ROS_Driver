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
#ifndef UR_DRIVER_DATA_PACKAGE_PUBLISHER_H_INCLUDED
#define UR_DRIVER_DATA_PACKAGE_PUBLISHER_H_INCLUDED

#include <ur_robot_driver/rtde/data_package.h>
#include <ur_robot_driver/ros/data_field_publisher.h>
#include <std_msgs/Int32.h>

namespace ur_driver
{
namespace rtde_interface
{
class DataPackagePublisher
{
public:
  DataPackagePublisher() = delete;

  DataPackagePublisher(const std::vector<std::string>& recipe, ros::NodeHandle& nh) : recipe_(recipe)
  {
    for (auto str : recipe)
    {
      publishers_.push_back(DataFieldPublisher::createFromString(str, nh));
    }
  }

  void publishData(const std::unique_ptr<DataPackage>& data_package)
  {
    for (auto const& i : publishers_)
    {
      i->publish(data_package);
    }
  }

private:
  std::vector<std::string> recipe_;
  std::list<std::unique_ptr<DataFieldPublisher>> publishers_;
};

}  // namespace rtde_interface
}  // namespace ur_driver

#endif  // ifndef UR_DRIVER_DATA_PACKAGE_PUBLISHER_H_INCLUDED
