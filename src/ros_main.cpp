#include <ros/ros.h>
#include <chrono>
#include <cstdlib>
#include <string>
#include <thread>

#include "ur_modern_driver/log.h"
#include "ur_modern_driver/pipeline.h"
#include "ur_modern_driver/ros/action_server.h"
#include "ur_modern_driver/ros/controller.h"
#include "ur_modern_driver/ros/io_service.h"
#include "ur_modern_driver/ros/mb_publisher.h"
#include "ur_modern_driver/ros/rt_publisher.h"
#include "ur_modern_driver/ros/service_stopper.h"
#include "ur_modern_driver/ros/trajectory_follower.h"
#include "ur_modern_driver/ur/commander.h"
#include "ur_modern_driver/ur/factory.h"
#include "ur_modern_driver/ur/messages.h"
#include "ur_modern_driver/ur/parser.h"
#include "ur_modern_driver/ur/producer.h"
#include "ur_modern_driver/ur/rt_state.h"
#include "ur_modern_driver/ur/state.h"

static const std::string IP_ADDR_ARG("~robot_ip_address");
static const std::string REVERSE_PORT_ARG("~reverse_port");
static const std::string ROS_CONTROL_ARG("~use_ros_control");
static const std::string MAX_VEL_CHANGE_ARG("~max_vel_change");
static const std::string PREFIX_ARG("~prefix");
static const std::string BASE_FRAME_ARG("~base_frame");
static const std::string TOOL_FRAME_ARG("~tool_frame");
static const std::string JOINT_NAMES_PARAM("hardware_interface/joints");

static const std::vector<std::string> DEFAULT_JOINTS = { "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
                                                         "wrist_1_joint",      "wrist_2_joint",       "wrist_3_joint" };

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
  double max_vel_change;
  int32_t reverse_port;
  bool use_ros_control;
};

bool parse_args(ProgArgs &args)
{
  if (!ros::param::get(IP_ADDR_ARG, args.host))
  {
    LOG_ERROR("robot_ip_address parameter must be set!");
    return false;
  }
  ros::param::param(REVERSE_PORT_ARG, args.reverse_port, int32_t(50001));
  ros::param::param(MAX_VEL_CHANGE_ARG, args.max_vel_change, 15.0);  // rad/s
  ros::param::param(MAX_VEL_CHANGE_ARG, args.max_velocity, 10.0);
  ros::param::param(ROS_CONTROL_ARG, args.use_ros_control, false);
  ros::param::param(PREFIX_ARG, args.prefix, std::string());
  ros::param::param(BASE_FRAME_ARG, args.base_frame, args.prefix + "base_link");
  ros::param::param(TOOL_FRAME_ARG, args.tool_frame, args.prefix + "tool0_controller");
  ros::param::param(JOINT_NAMES_PARAM, args.joint_names, DEFAULT_JOINTS);
  return true;
}

std::string getLocalIPAccessibleFromHost(std::string &host)
{
  URStream stream(host, UR_RT_PORT);
  return stream.connect() ? stream.getIP() : std::string();
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "ur_driver");

  ProgArgs args;
  if (!parse_args(args))
  {
    return EXIT_FAILURE;
  }

  //Add prefix to joint names
  std::transform (args.joint_names.begin(), args.joint_names.end(), args.joint_names.begin(),
        [&args](std::string name){return args.prefix + name;});

  std::string local_ip(getLocalIPAccessibleFromHost(args.host));

  URFactory factory(args.host);
  vector<Service *> services;

  // RT packets
  auto rt_parser = factory.getRTParser();
  URStream rt_stream(args.host, UR_RT_PORT);
  URProducer<RTPacket> rt_prod(rt_stream, *rt_parser);
  RTPublisher rt_pub(args.prefix, args.base_frame, args.tool_frame, args.use_ros_control);
  auto rt_commander = factory.getCommander(rt_stream);
  vector<IConsumer<RTPacket> *> rt_vec{ &rt_pub };

  TrajectoryFollower traj_follower(*rt_commander, local_ip, args.reverse_port, factory.isVersion3());

  ROSController *controller(nullptr);
  ActionServer *action_server(nullptr);
  if (args.use_ros_control)
  {
    LOG_INFO("ROS control enabled");
    controller = new ROSController(*rt_commander, traj_follower, args.joint_names, args.max_vel_change);
    rt_vec.push_back(controller);
    services.push_back(controller);
  }
  else
  {
    LOG_INFO("ActionServer enabled");
    action_server = new ActionServer(traj_follower, args.joint_names, args.max_velocity);
    rt_vec.push_back(action_server);
    services.push_back(action_server);
  }

  MultiConsumer<RTPacket> rt_cons(rt_vec);
  Pipeline<RTPacket> rt_pl(rt_prod, rt_cons);

  // Message packets
  auto state_parser = factory.getStateParser();
  URStream state_stream(args.host, UR_SECONDARY_PORT);
  URProducer<StatePacket> state_prod(state_stream, *state_parser);
  MBPublisher state_pub;

  ServiceStopper service_stopper(services);

  vector<IConsumer<StatePacket> *> state_vec{ &state_pub, &service_stopper };
  MultiConsumer<StatePacket> state_cons(state_vec);
  Pipeline<StatePacket> state_pl(state_prod, state_cons);

  LOG_INFO("Starting main loop");

  rt_pl.run();
  state_pl.run();

  auto state_commander = factory.getCommander(state_stream);
  IOService io_service(*state_commander);

  if (action_server)
    action_server->start();

  ros::spin();

  LOG_INFO("ROS stopping, shutting down pipelines");

  rt_pl.stop();
  state_pl.stop();

  if (controller)
    delete controller;

  LOG_INFO("Pipelines shutdown complete");

  return EXIT_SUCCESS;
}
