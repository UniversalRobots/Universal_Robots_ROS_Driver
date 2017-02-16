#pragma once

#include <cstdlib>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/WrenchStamped.h>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <vector>

#include "ur_modern_driver/ur/consumer.h"

using namespace ros;

const std::string JOINTS[] = {
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint"
};

class RTPublisher : public URRTPacketConsumer {
private:
    NodeHandle _nh;
    Publisher _joint_pub;
    Publisher _wrench_pub;
    Publisher _tool_vel_pub;
    std::vector<std::string> _joint_names;
    std::string _base_frame;

    bool publish_joints(RTShared& packet, ros::Time& t);
    bool publish_wrench(RTShared& packet, ros::Time& t);
    bool publish_tool(RTShared& packet, ros::Time& t);

    bool publish(RTShared& packet);

public:
    RTPublisher(std::string& joint_prefix, std::string& base_frame)
        : _joint_pub(_nh.advertise<sensor_msgs::JointState>("joint_states", 1))
        , _wrench_pub(_nh.advertise<geometry_msgs::WrenchStamped>("wrench", 1))
        , _tool_vel_pub(_nh.advertise<geometry_msgs::TwistStamped>("tool_velocity", 1))
        , _base_frame(base_frame)
    {
        for (auto const& j : JOINTS) {
            _joint_names.push_back(joint_prefix + j);
        }
    }

    virtual bool consume(RTState_V1_6__7& state);
    virtual bool consume(RTState_V1_8& state);
    virtual bool consume(RTState_V3_0__1& state);
    virtual bool consume(RTState_V3_2__3& state);

    virtual void setup_consumer() {}
    virtual void teardown_consumer() {}
    virtual void stop_consumer() {}
};