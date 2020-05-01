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
 * \date    2019-01-10
 *
 */
//----------------------------------------------------------------------

#include <ur_calibration/calibration_consumer.h>

#include <ur_robot_driver/comm/parser.h>
#include <ur_robot_driver/comm/pipeline.h>
#include <ur_robot_driver/comm/producer.h>
#include <ur_robot_driver/comm/stream.h>
#include <ur_robot_driver/primary/package_header.h>
#include <ur_robot_driver/primary/primary_parser.h>

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
    std::string output_package_name;
    try
    {
      // The robot's IP address
      robot_ip_ = getRequiredParameter<std::string>("robot_ip");

      // The target file where the calibration data is written to
      output_filename_ = getRequiredParameter<std::string>("output_filename");
    }
    catch (const ParamaterMissingException& e)
    {
      ROS_FATAL_STREAM(e.what());
      exit(1);
    }
  }

  virtual ~CalibrationCorrection() = default;

  void run()
  {
    comm::URStream<PrimaryPackage> stream(robot_ip_, UR_PRIMARY_PORT);
    primary_interface::PrimaryParser parser;
    comm::URProducer<PrimaryPackage> prod(stream, parser);
    CalibrationConsumer consumer;

    comm::INotifier notifier;

    comm::Pipeline<PrimaryPackage> pipeline(prod, &consumer, "Pipeline", notifier);
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

    fs::path out_path = fs::complete(output_filename_);

    fs::path dst_path = out_path.parent_path();
    if (!fs::exists(dst_path))
    {
      ROS_ERROR_STREAM("Parent folder " << dst_path << " does not exist.");
      return false;
    }
    ROS_INFO_STREAM("Writing calibration data to " << out_path);
    if (fs::exists(output_filename_))
    {
      ROS_WARN_STREAM("Output file " << output_filename_ << " already exists. Overwriting.");
    }
    std::ofstream file(output_filename_);
    if (file.is_open())
    {
      file << *calibration_data_;
    }
    else
    {
      ROS_ERROR_STREAM("Failed writing the file. Do you have the correct rights?");
      return false;
    }
    ROS_INFO_STREAM("Wrote output.");

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
  std::string output_filename_;

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
  catch (const std::exception& e)
  {
    ROS_ERROR_STREAM(e.what());
  }
  return 0;
}
