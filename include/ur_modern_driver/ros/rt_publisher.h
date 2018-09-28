#pragma once

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/WrenchStamped.h>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/Temperature.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <cstdlib>
#include <vector>

#include "ur_modern_driver/ur/consumer.h"

using namespace ros;
using namespace tf;

const std::string JOINTS[] = { "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
                               "wrist_1_joint",      "wrist_2_joint",       "wrist_3_joint" };

class RTPublisher : public URRTPacketConsumer
{
private:
  NodeHandle nh_;
  Publisher joint_pub_;
  Publisher wrench_pub_;
  Publisher tool_vel_pub_;
  Publisher joint_temperature_pub_;
  TransformBroadcaster transform_broadcaster_;
  std::vector<std::string> joint_names_;
  std::string base_frame_;
  std::string tool_frame_;
  bool temp_only_;

  bool publishJoints(RTShared& packet, Time& t);
  bool publishWrench(RTShared& packet, Time& t);
  bool publishTool(RTShared& packet, Time& t);
  bool publishTransform(RTShared& packet, Time& t);
  bool publishTemperature(RTShared& packet, Time& t);

  bool publish(RTShared& packet);

public:
  RTPublisher(std::string& joint_prefix, std::string& base_frame, std::string& tool_frame, bool temp_only = false)
    : joint_pub_(nh_.advertise<sensor_msgs::JointState>("joint_states", 1))
    , wrench_pub_(nh_.advertise<geometry_msgs::WrenchStamped>("wrench", 1))
    , tool_vel_pub_(nh_.advertise<geometry_msgs::TwistStamped>("tool_velocity", 1))
    , joint_temperature_pub_(nh_.advertise<sensor_msgs::Temperature>("joint_temperature", 1))
    , base_frame_(base_frame)
    , tool_frame_(tool_frame)
    , temp_only_(temp_only)
  {
    for (auto const& j : JOINTS)
    {
      joint_names_.push_back(joint_prefix + j);
    }
  }

  virtual bool consume(RTState_V1_6__7& state);
  virtual bool consume(RTState_V1_8& state);
  virtual bool consume(RTState_V3_0__1& state);
  virtual bool consume(RTState_V3_2__3& state);

  virtual void setupConsumer()
  {
  }
  virtual void teardownConsumer()
  {
  }
  virtual void stopConsumer()
  {
  }
};
