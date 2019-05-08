// this is for emacs file handling -*- mode: c++; indent-tabs-mode: nil -*-

// -- BEGIN LICENSE BLOCK ----------------------------------------------
// Copyright 2019 FZI Forschungszentrum Informatik
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// Many parts from this (Most of the URScript program) comes from the ur_modern_driver
// Copyright 2017, 2018 Simon Rasmussen (refactor)
// Copyright 2015, 2016 Thomas Timm Andersen (original version)
//
// -- END LICENSE BLOCK ------------------------------------------------

//----------------------------------------------------------------------
/*!\file
 *
 * \author  Felix Mauch mauch@fzi.de
 * \date    2019-04-11
 *
 */
//----------------------------------------------------------------------

#include "ur_rtde_driver/ur/ur_driver.h"
#include <memory>

namespace ur_driver
{
static const int32_t MULT_JOINTSTATE_ = 1000000;
static const std::string JOINT_STATE_REPLACE("{{JOINT_STATE_REPLACE}}");
static const std::string SERVO_J_REPLACE("{{SERVO_J_REPLACE}}");
static const std::string SERVER_IP_REPLACE("{{SERVER_IP_REPLACE}}");
static const std::string SERVER_PORT_REPLACE("{{SERVER_PORT_REPLACE}}");
static const std::string POSITION_PROGRAM = R"(
def myProg():
  textmsg("hello")
  MULT_jointstate = {{JOINT_STATE_REPLACE}}

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
        servoj(q, {{SERVO_J_REPLACE}})
        servoj(q, t=0.008, lookahead_time=0.03, gain=750)
      else:
        sync()
      end
    end
  end
  socket_open("{{SERVER_IP_REPLACE}}", {{SERVER_PORT_REPLACE}})

  thread_servo = run servoThread()
  keepalive = -2
  params_mult = socket_read_binary_integer(6+1)
  keepalive = params_mult[7]
  while keepalive > 0:
    params_mult = socket_read_binary_integer(6+1)
    keepalive = params_mult[7]
    if keepalive > 0:
      if params_mult[0] > 0:
        q = [params_mult[1] / MULT_jointstate, params_mult[2] / MULT_jointstate, params_mult[3] / MULT_jointstate, params_mult[4] / MULT_jointstate, params_mult[5] / MULT_jointstate, params_mult[6] / MULT_jointstate]
        set_servo_setpoint(q)
      end
    end
  end
  sleep(.1)
  socket_close()
  kill thread_servo
end
)";

ur_driver::UrDriver::UrDriver(const std::string& ROBOT_IP)
  : servoj_time_(0.002), servoj_gain_(750), servoj_lookahead_time_(0.03)
{
  ROS_INFO_STREAM("Initializing RTDE client");
  rtde_client_.reset(new rtde_interface::RTDEClient(ROBOT_IP, notifier_));

  if (!rtde_client_->init())
  {
    throw std::runtime_error("initialization went wrong");  // TODO: be less harsh
  }

  rtde_frequency_ = rtde_client_->getMaxFrequency();
  servoj_time_ = 1.0 / rtde_frequency_;

  comm::URStream<rtde_interface::PackageHeader> stream(ROBOT_IP, 30001);
  stream.connect();
  std::string local_ip = stream.getIP();

  uint32_t reverse_port = 50001;  // TODO: Make this a parameter

  std::string prog = POSITION_PROGRAM;
  prog.replace(prog.find(JOINT_STATE_REPLACE), JOINT_STATE_REPLACE.length(), std::to_string(MULT_JOINTSTATE_));
  std::ostringstream out;
  out << "t=" << std::fixed << std::setprecision(4) << servoj_time_;
  out << ", lookahead_time=" << servoj_lookahead_time_ << ", gain=" << servoj_gain_;
  prog.replace(prog.find(SERVO_J_REPLACE), SERVO_J_REPLACE.length(), out.str());
  prog.replace(prog.find(SERVER_IP_REPLACE), SERVER_IP_REPLACE.length(), local_ip);
  prog.replace(prog.find(SERVER_PORT_REPLACE), SERVER_PORT_REPLACE.length(), std::to_string(reverse_port));
  size_t len = prog.size();
  const uint8_t* data = reinterpret_cast<const uint8_t*>(prog.c_str());
  size_t written;

  if (stream.write(data, len, written))
  {
    LOG_INFO("Sent program to robot");
  }
  else
  {
    LOG_ERROR("Could not send program to robot");
  }

  reverse_interface_.reset(new comm::ReverseInterface(reverse_port));

  ROS_INFO_STREAM("Created reverse interface");

  rtde_client_->start();  // TODO: Add extra start method (also to HW-Interface)
  ROS_INFO_STREAM("Initialization done");
}

std::unique_ptr<rtde_interface::DataPackage> ur_driver::UrDriver::getDataPackage()
{
  // TODO: This goes into the rtde_client
  std::unique_ptr<comm::URPackage<rtde_interface::PackageHeader>> urpackage;
  uint32_t period_ms = (1.0 / rtde_frequency_) * 1000;
  std::chrono::milliseconds timeout(period_ms);
  if (rtde_client_->getDataPackage(urpackage, timeout))
  {
    rtde_interface::DataPackage* tmp = dynamic_cast<rtde_interface::DataPackage*>(urpackage.get());
    if (tmp != nullptr)
    {
      urpackage.release();
      return std::unique_ptr<rtde_interface::DataPackage>(tmp);
    }
  }
  return nullptr;
}

bool UrDriver::writeJointCommand(const vector6d_t& values)
{
  if (reverse_interface_)
  {
    reverse_interface_->write(values);
  }
  else
  {
    return false;
  }

  return true;
}
}  // namespace ur_driver
