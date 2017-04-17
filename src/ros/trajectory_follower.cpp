#include <endian.h>
#include "ur_modern_driver/ros/trajectory_follower.h"


TrajectoryFollower::TrajectoryFollower(URCommander &commander, int reverse_port, bool version_3)
  : running_(false)
  , commander_(commander)
  , server_(reverse_port)
  , program_(buildProgram(version_3))
{
}
static const std::string JOINT_STATE_REPLACE("{{JOINT_STATE_REPLACE}}");
static const std::string SERVO_J_REPLACE("{{SERVO_J_REPLACE}}");
static const std::string POSITION_PROGRAM = R"(
def driverProg():
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
std::string TrajectoryFollower::buildProgram(bool version_3)
{
  std::string res(POSITION_PROGRAM);
  size_t js_idx = POSITION_PROGRAM.find(JOINT_STATE_REPLACE);
  size_t sj_idx = POSITION_PROGRAM.find(SERVO_J_REPLACE);


  std::ostringstream out;
  out << "t=" << std::fixed << std::setprecision(4) << servoj_time_;
  
  if(version_3)
    out << ", lookahead_time=" << servoj_lookahead_time_ << ", gain=" << servoj_gain_;

  res.replace(js_idx, JOINT_STATE_REPLACE.length(), std::to_string(MULT_JOINTSTATE_));
  res.replace(sj_idx, SERVO_J_REPLACE.length(), out.str());
  return res;
}

bool TrajectoryFollower::start()
{
  if(running_)
    return true; //not sure

  //TODO
  std::string prog(""); // buildProg();
  if(!commander_.uploadProg(prog))
    return false;

  stream_ = std::move(server_.accept()); //todo: pointer instead?
  return (running_ = true);
}

bool TrajectoryFollower::execute(std::array<double, 6> &positions, bool keep_alive)
{
  if(!running_)
    return false;

  last_positions_ = positions;

  uint8_t buf[sizeof(uint32_t)*7];
  uint8_t *idx = buf;
  
  for(auto const& pos : positions)
  {
    int32_t val = static_cast<int32_t>(pos * MULT_JOINTSTATE_);
    val = htobe32(val);
    idx += append(idx, val);
  }

  int32_t val = htobe32(static_cast<int32_t>(keep_alive));
  append(idx, val);

  ssize_t res = stream_.send(buf, sizeof(buf));
  return res > 0 && res == sizeof(buf);
}

bool TrajectoryFollower::execute(std::array<double, 6> &positions)
{
  return execute(positions, true);
}

void TrajectoryFollower::stop()
{
  if(!running_)
    return;

  std::array<double, 6> empty;
  execute(empty, false);

  stream_.disconnect();
  running_ = false;
}