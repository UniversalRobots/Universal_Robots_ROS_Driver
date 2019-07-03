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
 * \date    2019-01-10
 *
 */
//----------------------------------------------------------------------

#include <ur_calibration/calibration_consumer.h>

#include <ur_rtde_driver/comm/parser.h>
#include <ur_rtde_driver/comm/pipeline.h>
#include <ur_rtde_driver/comm/producer.h>
#include <ur_rtde_driver/comm/stream.h>
#include <ur_rtde_driver/primary/package_header.h>
#include <ur_rtde_driver/primary/primary_parser.h>

#include <sensor_msgs/JointState.h>
#include <tf/transform_listener.h>
#include <ros/package.h>

#include <boost/filesystem.hpp>

namespace fs = boost::filesystem;

using namespace ur_driver;
using namespace primary_interface;
using namespace ur_calibration;

class ParamaterMissingException : public ros::Exception
{
public:
  ParamaterMissingException(const std::string& name)
    : Exception("Cannot find required parameter " + name + " on the parameter server.")
  {
  }
};

class CalibrationCorrection
{
public:
  CalibrationCorrection(const ros::NodeHandle& nh) : nh_(nh)
  {
    subfolder_ = nh_.param<std::string>("subfolder_name", "etc");
    std::string output_package_name;
    try
    {
      // The robot's IP address
      robot_ip_ = getRequiredParameter<std::string>("robot_ip");

      // Name with which the robot will be referenced. Will be used for the filename the calibration
      // data is stored in. This can be any arbitrary name. If left empty, the robot's serial number
      // will be used.
      robot_name_ = getRequiredParameter<std::string>("robot_name");

      // The resulting parameter file will be stored inside
      // <output_package_name>/<subfolder>/<robot_name>_calibration.yaml
      output_package_name = getRequiredParameter<std::string>("output_package_name");
    }
    catch (const ParamaterMissingException& e)
    {
      ROS_FATAL_STREAM(e.what());
      exit(1);
    }
    package_path_ = ros::package::getPath(output_package_name);
    if (package_path_ == "")
    {
      ROS_FATAL_STREAM("Could not find package " << output_package_name << ". Cannot save output file there.");
      exit(1);
    }
  }

  virtual ~CalibrationCorrection() = default;

  void run()
  {
    comm::URStream<PackageHeader> stream(robot_ip_, UR_PRIMARY_PORT);
    primary_interface::PrimaryParser parser;
    comm::URProducer<PackageHeader> prod(stream, parser);
    CalibrationConsumer consumer;

    comm::INotifier notifier;

    comm::Pipeline<PackageHeader> pipeline(prod, consumer, "Pipeline", notifier);
    pipeline.run();
    while (!consumer.isCalibrated())
    {
      ros::Duration(0.1).sleep();
    }
    calibration_data_.reset(new YAML::Node);
    *calibration_data_ = consumer.getCalibrationParameters();
  }

  bool writeCalibrationData()
  {
    if (calibration_data_ == nullptr)
    {
      ROS_ERROR_STREAM("Calibration data not yet set.");
      return false;
    }

    ROS_INFO_STREAM("Writing calibration data");
    fs::path dst_path = fs::path(package_path_) / fs::path(subfolder_);
    if (!fs::exists(dst_path))
    {
      try
      {
        fs::create_directory(dst_path);
      }
      catch (const boost::filesystem::filesystem_error& e)
      {
        ROS_ERROR_STREAM("Could not create " << dst_path << ". Reason: " << e.what());
        return false;
      }
    }
    fs::path output_filename = dst_path / fs::path(robot_name_ + "_calibration.yaml");
    if (fs::exists(output_filename))
    {
      ROS_WARN_STREAM("Output file " << output_filename << " already exists. Overwriting.");
    }
    std::ofstream file(output_filename.string());
    if (file.is_open())
    {
      file << *calibration_data_;
    }
    else
    {
      ROS_ERROR_STREAM("Failed writing the file. Do you have the correct rights?");
      return false;
    }
    ROS_INFO_STREAM("Wrote output to " << output_filename);

    return true;
  }

private:
  template <typename T>
  T getRequiredParameter(const std::string& param_name) const
  {
    T ret_val;
    if (nh_.hasParam(param_name))
    {
      nh_.getParam(param_name, ret_val);
    }
    else
    {
      throw ParamaterMissingException(nh_.resolveName(param_name));
    }

    return ret_val;
  }

  ros::NodeHandle nh_;
  std::string robot_ip_;
  std::string robot_name_;
  std::string package_path_;
  std::string subfolder_;

  std::unique_ptr<YAML::Node> calibration_data_;
};

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "ur_calibration");
  ros::NodeHandle nh("~");

  try
  {
    CalibrationCorrection my_calibration_correction(nh);
    my_calibration_correction.run();
    if (!my_calibration_correction.writeCalibrationData())
    {
      ROS_ERROR_STREAM("Failed writing calibration data. See errors above for details.");
      return -1;
    }
    ROS_INFO("Calibration correction done");
  }
  catch (const UrException& e)
  {
    ROS_ERROR_STREAM(e.what());
  }
  return 0;
}
