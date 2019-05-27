// this is for emacs file handling -*- mode: c++; indent-tabs-mode: nil -*-

// -- BEGIN LICENSE BLOCK ----------------------------------------------
// -- END LICENSE BLOCK ------------------------------------------------

//----------------------------------------------------------------------
/*!\file
 *
 * \author  Felix Mauch mauch@fzi.de
 * \date    2018-12-11
 *
 */
//----------------------------------------------------------------------
#include <ur_rtde_driver/comm/producer.h>
#include <ur_rtde_driver/comm/shell_consumer.h>
#include <ur_rtde_driver/comm/stream.h>
#include <ur_rtde_driver/comm/parser.h>
#include <ur_rtde_driver/comm/pipeline.h>

#include <ur_rtde_driver/primary/package_header.h>
#include <ur_rtde_driver/primary/primary_parser.h>

static const int UR_PRIMARY_PORT = 30001;
static const int UR_SECONDARY_PORT = 30002;
static const int UR_RT_PORT = 30003;

using namespace ur_driver;
using namespace primary_interface;

int main(int argc, char* argv[])
{
  std::string ROBOT_IP = "192.168.56.101";
  // std::string ROBOT_IP = "192.168.0.104";
  comm::URStream<PackageHeader> stream(ROBOT_IP, UR_PRIMARY_PORT);

  if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug))
  {
    ros::console::notifyLoggerLevelsChanged();
  }

  primary_interface::PrimaryParser parser;
  comm::URProducer<PackageHeader> prod(stream, parser);
  comm::ShellConsumer<PackageHeader> consumer;

  comm::INotifier notifier;

  comm::Pipeline<PackageHeader> pipeline(prod, consumer, "Pipeline", notifier);
  LOG_INFO("Running now");
  pipeline.run();
  while (true)
  {
    sleep(1);
    // LOG_INFO("Still running");
  }

  return 0;
}
