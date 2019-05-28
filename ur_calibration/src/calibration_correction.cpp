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

using namespace ur_driver;
using namespace primary_interface;
using namespace ur_calibration;

static const int UR_PRIMARY_PORT = 30001;

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "ur_calibration");
  ros::NodeHandle nh;

  std::string robot_ip = "192.168.56.101";
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
  file.open("test.yaml");
  YAML::Node calibration_parameters = consumer.getCalibrationParameters();
  file << calibration_parameters;
  file.close();

  ROS_INFO_STREAM("Corrected calibration: " << std::endl << calibration_parameters);

  return 0;
}
