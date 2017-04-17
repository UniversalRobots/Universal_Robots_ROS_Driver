#include "ur_modern_driver/ros/hardware_interface.h"

JointInterface::JointInterface(std::vector<std::string> &joint_names)
{
    for (size_t i = 0; i < 6; i++)
    {
        registerHandle(hardware_interface::JointStateHandle(joint_names[i], &positions_[i], &velocities_[i], &efforts_[i]));
    }
}

void JointInterface::update(RTShared &packet)
{
    positions_ = packet.q_actual;
    velocities_ = packet.qd_actual;
    efforts_ = packet.i_actual;
}

WrenchInterface::WrenchInterface()
{
    registerHandle(hardware_interface::ForceTorqueSensorHandle("wrench", "", tcp_.begin(), tcp_.begin() + 3));
}

void WrenchInterface::update(RTShared &packet)
{
    tcp_ = packet.tcp_force;
}

VelocityInterface::VelocityInterface(URCommander &commander, hardware_interface::JointStateInterface &js_interface, std::vector<std::string> &joint_names, double max_vel_change)
    : commander_(commander), max_vel_change_(max_vel_change)
{
    for (size_t i = 0; i < 6; i++)
    {
        registerHandle(JointHandle(js_interface.getHandle(joint_names[i]), &velocity_cmd_[i]));
    }
}

bool VelocityInterface::write()
{
    for (size_t i = 0; i < 6; i++)
    {
        double prev = prev_velocity_cmd_[i];
        double lo = prev - max_vel_change_;
        double hi = prev + max_vel_change_;
        // clamp value to ±max_vel_change
        prev_velocity_cmd_[i] = std::max(lo, std::min(velocity_cmd_[i], hi));
    }

    return commander_.speedj(prev_velocity_cmd_, max_vel_change_);
}


PositionInterface::  PositionInterface(URCommander &commander, hardware_interface::JointStateInterface &js_interface, std::vector<std::string> &joint_names)
    : commander_(commander)
{
    for (size_t i = 0; i < 6; i++)
    {
        registerHandle(JointHandle(js_interface.getHandle(joint_names[i]), &position_cmd_[i]));
    }
}

bool PositionInterface::write()
{

}

void PositionInterface::start()
{

}

void PositionInterface::stop()
{

}