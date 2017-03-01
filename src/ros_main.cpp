#include <ros/ros.h>
#include <chrono>
#include <cstdlib>
#include <string>
#include <thread>

#include "ur_modern_driver/log.h"
#include "ur_modern_driver/pipeline.h"
#include "ur_modern_driver/ros/rt_publisher.h"
#include "ur_modern_driver/ur/factory.h"
#include "ur_modern_driver/ur/messages.h"
#include "ur_modern_driver/ur/parser.h"
#include "ur_modern_driver/ur/producer.h"
#include "ur_modern_driver/ur/rt_state.h"
#include "ur_modern_driver/ur/state.h"

static const std::string IP_ADDR_ARG("~robot_ip_address");
static const std::string REVERSE_PORT_ARG("~reverse_port");
static const std::string SIM_TIME_ARG("~use_sim_time");
static const std::string PREFIX_ARG("~prefix");
static const std::string BASE_FRAME_ARG("~base_frame");
static const std::string TOOL_FRAME_ARG("~tool_frame");

struct ProgArgs
{
public:
  std::string host;
  std::string prefix;
  std::string base_frame;
  std::string tool_frame;
  int32_t reverse_port;
  bool use_sim_time;
};

bool parse_args(ProgArgs& args)
{
  if (!ros::param::get(IP_ADDR_ARG, args.host))
  {
    LOG_ERROR("robot_ip_address parameter must be set!");
    return false;
  }
  ros::param::param(REVERSE_PORT_ARG, args.reverse_port, int32_t(50001));
  ros::param::param(SIM_TIME_ARG, args.use_sim_time, false);
  ros::param::param(PREFIX_ARG, args.prefix, std::string());
  ros::param::param(BASE_FRAME_ARG, args.base_frame, args.prefix + "base_link");
  ros::param::param(TOOL_FRAME_ARG, args.tool_frame, args.prefix + "tool0_controller");
  return true;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "ur_driver");

  ProgArgs args;
  if (!parse_args(args))
  {
    return EXIT_FAILURE;
  }

  URFactory factory(args.host);
  auto parser = factory.getRTParser();

  URStream s2(args.host, 30003);
  URProducer<RTPacket> p2(s2, *parser);
  RTPublisher pub(args.prefix, args.base_frame, args.tool_frame);

  Pipeline<RTPacket> pl(p2, pub);

  pl.run();

  while (ros::ok())
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(33));
  }

  pl.stop();

  return EXIT_SUCCESS;
}