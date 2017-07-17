#pragma once
#include <controller_manager/controller_manager.h>
#include <hardware_interface/force_torque_sensor_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <ros/ros.h>
#include <atomic>
#include "ur_modern_driver/log.h"
#include "ur_modern_driver/ros/hardware_interface.h"
#include "ur_modern_driver/ros/service_stopper.h"
#include "ur_modern_driver/ur/commander.h"
#include "ur_modern_driver/ur/consumer.h"
#include "ur_modern_driver/ur/rt_state.h"

class ROSController : private hardware_interface::RobotHW, public URRTPacketConsumer, public Service
{
private:
  ros::NodeHandle nh_;
  ros::Time lastUpdate_;
  controller_manager::ControllerManager controller_;

  // state interfaces
  JointInterface joint_interface_;
  WrenchInterface wrench_interface_;
  // controller interfaces
  PositionInterface position_interface_;
  VelocityInterface velocity_interface_;

  // currently activated controller
  HardwareInterface* active_interface_;
  // map of switchable controllers
  std::map<std::string, HardwareInterface*> available_interfaces_;

  std::atomic<bool> service_enabled_;
  std::atomic<uint32_t> service_cooldown_;

  // helper functions to map interfaces
  template <typename T>
  void registerInterface(T* interface)
  {
    RobotHW::registerInterface<typename T::parent_type>(interface);
  }
  template <typename T>
  void registerControllereInterface(T* interface)
  {
    registerInterface(interface);
    available_interfaces_[T::INTERFACE_NAME] = interface;
  }

  void read(RTShared& state);
  bool update(RTShared& state);
  bool write();
  void reset();

public:
  ROSController(URCommander& commander, TrajectoryFollower& follower, std::vector<std::string>& joint_names,
                double max_vel_change);
  virtual ~ROSController()
  {
  }
  // from RobotHW
  void doSwitch(const std::list<hardware_interface::ControllerInfo>& start_list,
                const std::list<hardware_interface::ControllerInfo>& stop_list);
  // from URRTPacketConsumer
  virtual void setupConsumer();
  virtual bool consume(RTState_V1_6__7& state)
  {
    return update(state);
  }
  virtual bool consume(RTState_V1_8& state)
  {
    return update(state);
  }
  virtual bool consume(RTState_V3_0__1& state)
  {
    return update(state);
  }
  virtual bool consume(RTState_V3_2__3& state)
  {
    return update(state);
  }

  virtual void onRobotStateChange(RobotState state);
};