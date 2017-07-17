#include "ur_modern_driver/ros/hardware_interface.h"
#include "ur_modern_driver/log.h"

const std::string JointInterface::INTERFACE_NAME = "joint_state_controller";
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


const std::string WrenchInterface::INTERFACE_NAME = "force_torque_sensor_controller";
WrenchInterface::WrenchInterface()
{
  registerHandle(hardware_interface::ForceTorqueSensorHandle("wrench", "", tcp_.begin(), tcp_.begin() + 3));
}

void WrenchInterface::update(RTShared &packet)
{
  tcp_ = packet.tcp_force;
}


const std::string VelocityInterface::INTERFACE_NAME = "vel_based_pos_traj_controller";
VelocityInterface::VelocityInterface(URCommander &commander, hardware_interface::JointStateInterface &js_interface,
                                     std::vector<std::string> &joint_names, double max_vel_change)
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
    // clamp value to ±max_vel_change
    double prev = prev_velocity_cmd_[i];
    double lo = prev - max_vel_change_;
    double hi = prev + max_vel_change_;
    prev_velocity_cmd_[i] = std::max(lo, std::min(velocity_cmd_[i], hi));
  }
  return commander_.speedj(prev_velocity_cmd_, max_vel_change_);
}

void VelocityInterface::reset()
{
  for (auto &val : prev_velocity_cmd_)
  {
    val = 0;
  }
}

const std::string PositionInterface::INTERFACE_NAME = "pos_based_pos_traj_controller";
PositionInterface::PositionInterface(TrajectoryFollower &follower,
                                     hardware_interface::JointStateInterface &js_interface,
                                     std::vector<std::string> &joint_names)
  : follower_(follower)
{
  for (size_t i = 0; i < 6; i++)
  {
    registerHandle(JointHandle(js_interface.getHandle(joint_names[i]), &position_cmd_[i]));
  }
}

bool PositionInterface::write()
{
  return follower_.execute(position_cmd_);
}

void PositionInterface::start()
{
  follower_.start();
}

void PositionInterface::stop()
{
  follower_.stop();
}