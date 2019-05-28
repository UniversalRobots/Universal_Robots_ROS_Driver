// this is for emacs file handling -*- mode: c++; indent-tabs-mode: nil -*-

// -- BEGIN LICENSE BLOCK ----------------------------------------------
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

static const int UR_PRIMARY_PORT = 30001;

class ParamaterMissingException : public ros::Exception
{
public:
  ParamaterMissingException(const std::string& name)
    : Exception("Cannot find required parameter " + name + " on the parameter server.")
  {
  }
};

template <typename T>
T getRequiredParameter(const ros::NodeHandle& nh, const std::string& param_name)
{
  T ret_val;
  if (nh.hasParam(param_name))
  {
    nh.getParam(param_name, ret_val);
  }
  else
  {
    throw ParamaterMissingException(nh.resolveName(param_name));
  }

  return ret_val;
}

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "ur_calibration");
  ros::NodeHandle nh("~");

  std::string subfolder = nh.param<std::string>("subfolder_name", "etc");
  std::string robot_ip, robot_name, output_package_name;
  try
  {
    // The robot's IP address
    robot_ip = getRequiredParameter<std::string>(nh, "robot_ip");

    // Name with which the robot will be referenced. Will be used for the filename the calibration
    // data is stored in. This can be any arbitrary name. If left empty, the robot's serial number
    // will be used.
    robot_name = getRequiredParameter<std::string>(nh, "robot_name");

    // The resulting parameter file will be stored inside
    // <output_package_name>/subfolder/<robot_name>_calibration.yaml
    output_package_name = getRequiredParameter<std::string>(nh, "output_package_name");
  }
  catch (const ParamaterMissingException& e)
  {
    ROS_FATAL_STREAM(e.what());
    return -1;
  }

  std::string package_path = ros::package::getPath(output_package_name);
  if (package_path == "")
  {
    ROS_FATAL_STREAM("Could not find package " << output_package_name << ". Cannot save output file there.");
    return -1;
  }

  comm::URStream<PackageHeader> stream(robot_ip, UR_PRIMARY_PORT);
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
  pipeline.stop();

  std::ofstream file;
  fs::path dst_path = fs::path(package_path) / fs::path(subfolder);
  if (!fs::exists(dst_path))
  {
    fs::create_directory(dst_path);
  }
  fs::path output_filename = dst_path / fs::path(robot_name + "_calibration.yaml");
  if (fs::exists(output_filename))
  {
    ROS_WARN_STREAM("Output file " << output_filename << " already exists. Overwriting.");
  }
  file.open(output_filename.string());
  YAML::Node calibration_parameters = consumer.getCalibrationParameters();
  file << calibration_parameters;
  file.close();

  ROS_INFO_STREAM("Corrected calibration: " << std::endl << calibration_parameters);

  return 0;
}
