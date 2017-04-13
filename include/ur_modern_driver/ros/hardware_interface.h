#pragma once
#include <controller_manager/controller_manager.h>
#include <hardware_interface/force_torque_sensor_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <algorithm>
#include "ur_modern_driver/ur/commander.h"
#include "ur_modern_driver/ur/rt_state.h"

class HardwareInterface
{
public:
  virtual void write() = 0;
  virtual void start()
  {
  }
  virtual void stop()
  {
  }
};

using hardware_interface::JointHandle;
using hardware_interface::JointStateHandle;
using hardware_interface::JointStateInterface;

class JointInterface : public JointStateInterface
{
private:
  std::array<double, 6> velocities_, positions_, efforts_;

public:
  JointInterface(std::vector<std::string>& joint_names)
  {
    for (size_t i = 0; i < 6; i++)
    {
      registerHandle(JointStateHandle(joint_names[i], &positions_[i], &velocities_[i], &efforts_[i]));
    }
  }

  void update(RTShared& packet)
  {
    positions_ = packet.q_actual;
    velocities_ = packet.qd_actual;
    efforts_ = packet.i_actual;
  }
};

class WrenchInterface : public hardware_interface::ForceTorqueSensorInterface
{
  std::array<double, 6> tcp_;
public:
  WrenchInterface() 
  {
    registerHandle(hardware_interface::ForceTorqueSensorHandle("wrench", "", tcp_.begin(), tcp_.begin()+3));
  }

  void update(RTShared& packet)
  {
    tcp_ = packet.tcp_force;
  }
};

class VelocityInterface : public HardwareInterface, public hardware_interface::VelocityJointInterface
{
private:
  URCommander& commander_;
  std::array<double, 6> velocity_cmd_, prev_velocity_cmd_;
  double max_vel_change_;

public:
  VelocityInterface(URCommander& commander, JointStateInterface& js_interface, std::vector<std::string>& joint_names)
    : commander_(commander)
  {
    for (size_t i = 0; i < 6; i++)
    {
      registerHandle(JointHandle(js_interface.getHandle(joint_names[i]), &velocity_cmd_[i]));
    }
  }

  virtual void write()
  {
    for (size_t i = 0; i < 6; i++)
    {
      double prev = prev_velocity_cmd_[i];
      double lo = prev - max_vel_change_;
      double hi = prev + max_vel_change_;
      // clamp value to ±max_vel_change
      prev_velocity_cmd_[i] = std::max(lo, std::min(velocity_cmd_[i], hi));
    }

    //times 125???
    commander_.speedj(prev_velocity_cmd_, max_vel_change_ * 125);
  }
};

static const std::string POSITION_PROGRAM = R"(
def driverProg():
    MULT_jointstate = XXXXX

    SERVO_IDLE = 0
    SERVO_RUNNING = 1
    cmd_servo_state = SERVO_IDLE
    cmd_servo_q = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    def set_servo_setpoint(q):
        enter_critical
        cmd_servo_state = SERVO_RUNNING
        cmd_servo_q = q
        exit_critical
    end
    thread servoThread():
        state = SERVO_IDLE
        while True:
            enter_critical
            q = cmd_servo_q
            do_brake = False
            if (state == SERVO_RUNNING) and (cmd_servo_state == SERVO_IDLE):
                do_brake = True
            end
            state = cmd_servo_state
            cmd_servo_state = SERVO_IDLE
            exit_critical
            if do_brake:
                stopj(1.0)
                sync()
            elif state == SERVO_RUNNING:
                servoj(q, YYYYYYYY)
            else:
                sync()
            end
        end
    end

    thread_servo = run servoThread()
    keepalive = 1
    while keepalive > 0:
        params_mult = socket_read_binary_integer(6+1)
        if params_mult[0] > 0:
            q = [params_mult[1] / MULT_jointstate, params_mult[2] / MULT_jointstate, params_mult[3] / MULT_jointstate, params_mult[4] / MULT_jointstate, params_mult[5] / MULT_jointstate, params_mult[6] / MULT_jointstate]
            keepalive = params_mult[7]
            set_servo_setpoint(q)
        end
    end
    sleep(.1)
    socket_close()
    kill thread_servo
end
)";

class PositionInterface : public HardwareInterface, public hardware_interface::PositionJointInterface
{
private:
  URCommander& commander_;
  std::array<double, 6> position_cmd_;

public:
  PositionInterface(URCommander& commander, JointStateInterface& js_interface, std::vector<std::string>& joint_names)
    : commander_(commander)
  {
    for (size_t i = 0; i < 6; i++)
    {
      registerHandle(JointHandle(js_interface.getHandle(joint_names[i]), &position_cmd_[i]));
    }
  }

  virtual void write()
  {
  }
};