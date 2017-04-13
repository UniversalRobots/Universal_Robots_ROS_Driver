#include <ros/ros.h>
#include <chrono>
#include <cstdlib>
#include <string>
#include <thread>

#include "ur_modern_driver/log.h"
#include "ur_modern_driver/pipeline.h"
#include "ur_modern_driver/ros/rt_publisher.h"
#include "ur_modern_driver/ros/mb_publisher.h"
#include "ur_modern_driver/ros/io_service.h"
#include "ur_modern_driver/ros/ros_controller.h"
#include "ur_modern_driver/ur/commander.h"
#include "ur_modern_driver/ur/factory.h"
#include "ur_modern_driver/ur/messages.h"
#include "ur_modern_driver/ur/parser.h"
#include "ur_modern_driver/ur/producer.h"
#include "ur_modern_driver/ur/rt_state.h"
#include "ur_modern_driver/ur/state.h"

static const std::string IP_ADDR_ARG("~robot_ip_address");
static const std::string REVERSE_PORT_ARG("~reverse_port");
static const std::string SIM_TIME_ARG("~use_sim_time");
static const std::string ROS_CONTROL_ARG("~use_ros_control");
static const std::string PREFIX_ARG("~prefix");
static const std::string BASE_FRAME_ARG("~base_frame");
static const std::string TOOL_FRAME_ARG("~tool_frame");
static const std::string JOINT_NAMES_PARAM("hardware_interface/joints");

static const int UR_SECONDARY_PORT = 30002;
static const int UR_RT_PORT = 30003;

struct ProgArgs
{
public:
  std::string host;
  std::string prefix;
  std::string base_frame;
  std::string tool_frame;
  std::vector<std::string> joint_names;
  double max_acceleration;
  double max_velocity;
  int32_t reverse_port;
  bool use_sim_time;
  bool use_ros_control;
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
  ros::param::param(ROS_CONTROL_ARG, args.use_ros_control, false);
  ros::param::param(PREFIX_ARG, args.prefix, std::string());
  ros::param::param(BASE_FRAME_ARG, args.base_frame, args.prefix + "base_link");
  ros::param::param(TOOL_FRAME_ARG, args.tool_frame, args.prefix + "tool0_controller");
  ros::param::get(JOINT_NAMES_PARAM, args.joint_names);  
  return true;
}

#include "ur_modern_driver/event_counter.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "ur_driver");

  ProgArgs args;
  if (!parse_args(args))
  {
    return EXIT_FAILURE;
  }

  URFactory factory(args.host);
  //RT packets
  auto rt_parser = factory.getRTParser();
  URStream rt_stream(args.host, UR_RT_PORT);
  URProducer<RTPacket> rt_prod(rt_stream, *rt_parser);
  RTPublisher rt_pub(args.prefix, args.base_frame, args.tool_frame);
  EventCounter rt_ec;

  URCommander rt_commander(rt_stream);
  vector<IConsumer<RTPacket>*> rt_vec;

  if(args.use_ros_control)
  {
    rt_vec.push_back(new ROSController(rt_commander, args.joint_names));
  }

  //rt_vec.push_back(&rt_pub);

  MultiConsumer<RTPacket> rt_cons(rt_vec);
  Pipeline<RTPacket> rt_pl(rt_prod, rt_cons);

  //Message packets
  auto state_parser = factory.getStateParser();
  URStream state_stream(args.host, UR_SECONDARY_PORT);
  URProducer<StatePacket> state_prod(state_stream, *state_parser);
  MBPublisher state_pub;
  vector<IConsumer<StatePacket>*> state_vec{&state_pub};
  MultiConsumer<StatePacket> state_cons(state_vec);
  Pipeline<StatePacket> state_pl(state_prod, state_cons);

  LOG_INFO("Starting main loop");

  rt_pl.run();
  state_pl.run();

  URCommander state_commander(state_stream);
  IOService service(state_commander);

  ros::spin();

  rt_pl.stop();
  state_pl.stop();

  return EXIT_SUCCESS;
}