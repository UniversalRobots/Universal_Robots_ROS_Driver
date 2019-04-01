/*
 * Copyright 2017, 2018 Jarek Potiuk (low bandwidth trajectory follower)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "ur_modern_driver/ros/lowbandwidth_trajectory_follower.h"
#include <endian.h>
#include <ros/ros.h>
#include <cmath>

static const std::array<double, 6> EMPTY_VALUES = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };

static const std::string TIME_INTERVAL("{{TIME_INTERVAL}}");
static const std::string SERVOJ_TIME("{{SERVOJ_TIME}}");
static const std::string SERVOJ_TIME_WAITING("{{SERVOJ_TIME_WAITING}}");
static const std::string MAX_WAITING_TIME("{{MAX_WAITING_TIME}}");
static const std::string SERVOJ_GAIN("{{SERVOJ_GAIN}}");
static const std::string SERVOJ_LOOKAHEAD_TIME("{{SERVOJ_LOOKAHEAD_TIME}}");
static const std::string REVERSE_IP("{{REVERSE_IP}}");
static const std::string REVERSE_PORT("{{REVERSE_PORT}}");
static const std::string MAX_JOINT_DIFFERENCE("{{MAX_JOINT_DIFFERENCE}}");
static const std::string POSITION_PROGRAM = R"(
def driveRobotLowBandwidthTrajectory():
    global JOINT_NUM               = 6
    global TIME_INTERVAL           = {{TIME_INTERVAL}}
    global SERVOJ_TIME             = {{SERVOJ_TIME}}
    global SERVOJ_TIME_WAITING     = {{SERVOJ_TIME_WAITING}}
    global MAX_WAITING_TIME        = {{MAX_WAITING_TIME}}
    global EMPTY_VALUES            = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    global SERVOJ_GAIN             = {{SERVOJ_GAIN}}
    global SERVOJ_LOOKAHEAD_TIME   = {{SERVOJ_LOOKAHEAD_TIME}}
    global CONNECTION_NAME         = "reverse_connection"
    global REVERSE_IP              = "{{REVERSE_IP}}"
    global REVERSE_PORT            = {{REVERSE_PORT}}
    global MAX_JOINT_DIFFERENCE    = {{MAX_JOINT_DIFFERENCE}}
    global g_position_previous = EMPTY_VALUES
    global g_position_target   = EMPTY_VALUES
    global g_position_next     = EMPTY_VALUES
    global g_velocity_previous = EMPTY_VALUES
    global g_velocity_target   = EMPTY_VALUES
    global g_velocity_next     = EMPTY_VALUES
    global g_time_previous = 0.0
    global g_time_target   = 0.0
    global g_time_next     = 0.0
    global g_num_previous = -1
    global g_num_target   = -1
    global g_num_next     = -1
    global g_received_waypoints_number  = -1
    global g_requested_waypoints_number = -1
    global g_total_elapsed_time = 0
    global g_stopping = False
    def send_message(message):
        socket_send_string(message, CONNECTION_NAME)
        socket_send_byte(10, CONNECTION_NAME)
    end

    def is_waypoint_sentinel(waypoint):
        local l_previous_index = 2
        while l_previous_index < 1 + JOINT_NUM * 2 + 2:
            if waypoint[l_previous_index] != 0.0:
                return False
            end
            l_previous_index = l_previous_index + 1
        end
        return True
    end

    def is_final_position_reached(position):
        local l_current_position = get_actual_joint_positions()
        local l_index = 0
        while l_index < JOINT_NUM:
            if norm(position[l_index] - l_current_position[l_index]) > MAX_JOINT_DIFFERENCE:
                return False
            end
            l_index = l_index + 1
        end
        return True
    end

    def interpolate(time_within_segment, total_segment_time, start_pos, l_end_pos, l_start_vel, end_vel):
        local a = start_pos
        local b = l_start_vel
        local c = (-3 * a + 3 * l_end_pos - 2 * total_segment_time * b - total_segment_time * end_vel) / pow(total_segment_time, 2)
        local d = (2 * a - 2 * l_end_pos + total_segment_time * b + total_segment_time * end_vel) / pow(total_segment_time, 3)
        return a + b * time_within_segment + c * pow(time_within_segment, 2) + d * pow(time_within_segment, 3)
    end

    def add_next_waypoint(waypoint):
        enter_critical
        g_position_previous = g_position_target
        g_velocity_previous = g_velocity_target
        g_time_previous     = g_time_target
        g_num_previous      = g_num_target
        g_position_target   = g_position_next
        g_velocity_target   = g_velocity_next
        g_time_target       = g_time_next
        g_num_target        = g_num_next
        g_num_next = waypoint[1]
        g_position_next     = [waypoint[2], waypoint[3], waypoint[4], waypoint[5], waypoint[6], waypoint[7]]
        g_velocity_next     = [waypoint[8], waypoint[9], waypoint[10], waypoint[11], waypoint[12], waypoint[13]]
        g_time_next         = waypoint[14]
        g_received_waypoints_number = g_num_next
        exit_critical
    end

    thread controllingThread():
        local l_received_waypoints_number = -1
        local l_requested_waypoints_number = -1
        local l_stopped = False
        local l_current_position = get_actual_joint_positions()
        enter_critical
        g_requested_waypoints_number = 2
        exit_critical
        while True:
            enter_critical
            l_requested_waypoints_number = g_requested_waypoints_number
            exit_critical
            local l_max_waiting_time_left = MAX_WAITING_TIME
            while l_received_waypoints_number < l_requested_waypoints_number and l_max_waiting_time_left > 0:
                servoj(l_current_position,t=SERVOJ_TIME_WAITING,lookahead_time=SERVOJ_LOOKAHEAD_TIME,gain=SERVOJ_GAIN)
                enter_critical
                l_received_waypoints_number = g_received_waypoints_number
                exit_critical
                l_max_waiting_time_left = l_max_waiting_time_left - SERVOJ_TIME_WAITING
            end
            if l_max_waiting_time_left <= 0:
                textmsg("Closing the connection on waiting too long.")
                socket_close(CONNECTION_NAME)
                halt
            end
            enter_critical
            local l_start_pos = g_position_previous
            local l_start_vel = g_velocity_previous
            local l_start_time = g_time_previous
            local l_start_num= g_num_previous
            local l_end_pos = g_position_target
            local l_end_vel = g_velocity_target
            local l_end_time = g_time_target
            local l_end_num = g_num_target
            local l_total_elapsed_time = g_total_elapsed_time
            local l_stopping_after_next_interpolation = g_stopping
            g_requested_waypoints_number = g_requested_waypoints_number + 1
            exit_critical

            l_current_position = l_start_pos

            local l_total_segment_time = l_end_time - l_start_time

            while l_total_elapsed_time <= l_end_time:
                local l_segment_elapsed_time = l_total_elapsed_time - l_start_time
                j = 0
                while j < JOINT_NUM:
                    l_current_position[j] = interpolate(l_segment_elapsed_time, l_total_segment_time, l_start_pos[j], l_end_pos[j], l_start_vel[j], l_end_vel[j])
                    j = j + 1
                end
                servoj(l_current_position,t=SERVOJ_TIME,lookahead_time=SERVOJ_LOOKAHEAD_TIME,gain=SERVOJ_GAIN)
                enter_critical
                g_total_elapsed_time = g_total_elapsed_time + TIME_INTERVAL
                l_total_elapsed_time = g_total_elapsed_time
                exit_critical

            end
            if l_stopping_after_next_interpolation:
                while not is_final_position_reached(l_end_pos):
                    textmsg("Catching up on final position not reached first time.")
                    servoj(l_end_pos,t=SERVOJ_TIME,lookahead_time=SERVOJ_LOOKAHEAD_TIME,gain=SERVOJ_GAIN)
                end
                textmsg("Finishing the controlling thread. Final position reached.")
                break
            end
        end
    end

    thread sendingThread():
        local controlling_thread = run controllingThread()
        local l_sent_waypoints_number = -1
        local l_requested_waypoints_number = -1
        local l_stopping = False

        enter_critical
        l_requested_waypoints_number = g_requested_waypoints_number
        l_stopping = g_stopping
        exit_critical
        while not l_stopping:
            while l_sent_waypoints_number == l_requested_waypoints_number and not l_stopping:
                sleep(SERVOJ_TIME_WAITING)

                enter_critical
                l_requested_waypoints_number = g_requested_waypoints_number
                l_stopping = g_stopping
                exit_critical

            end
            if l_stopping:
                break
            end
            send_message(l_sent_waypoints_number + 1)
            l_sent_waypoints_number = l_sent_waypoints_number + 1
        end
        join controlling_thread
    end

    thread receivingThread():
        local sending_thread = run sendingThread()
        while True:
            waypoint_received = socket_read_ascii_float(14, CONNECTION_NAME)
            if waypoint_received[0] == 0:
                textmsg("Not received trajectory for the last 2 seconds. Quitting")
                enter_critical
                g_stopping = True
                exit_critical
                break
            elif waypoint_received[0] != JOINT_NUM * 2 + 2:
                textmsg("Received wrong number of floats in trajectory. This is certainly not OK.")
                textmsg(waypoint_received[0])
                enter_critical
                g_stopping = True
                exit_critical
                break
            elif is_waypoint_sentinel(waypoint_received):
                add_next_waypoint(waypoint_received)
                enter_critical
                g_stopping = True
                g_received_waypoints_number = g_received_waypoints_number + 1
                exit_critical
                break
            end
            add_next_waypoint(waypoint_received)
        end
        join sending_thread
    end
    socket_open(REVERSE_IP, REVERSE_PORT, CONNECTION_NAME)
    receiving_thread = run receivingThread()
    join receiving_thread
    socket_close(CONNECTION_NAME)
    textmsg("Exiting the program")
end

)";

LowBandwidthTrajectoryFollower::LowBandwidthTrajectoryFollower(URCommander &commander, std::string &reverse_ip,
                                                               int reverse_port, bool version_3)
  : running_(false)
  , commander_(commander)
  , server_(reverse_port)
  , time_interval_(0.008)
  , servoj_time_(0.008)
  , servoj_time_waiting_(0.001)
  , max_waiting_time_(2.0)
  , servoj_gain_(300.0)
  , servoj_lookahead_time_(0.03)
  , max_joint_difference_(0.01)
{
  ros::param::get("~time_interval", time_interval_);
  ros::param::get("~servoj_time", servoj_time_);
  ros::param::get("~servoj_time_waiting", servoj_time_waiting_);
  ros::param::get("~max_waiting_time", max_waiting_time_);
  ros::param::get("~servoj_gain", servoj_gain_);
  ros::param::get("~servoj_lookahead_time", servoj_lookahead_time_);
  ros::param::get("~max_joint_difference", max_joint_difference_);

  std::string res(POSITION_PROGRAM);
  std::ostringstream out;
  if (!version_3)
  {
    LOG_ERROR("Low Bandwidth Trajectory Follower only works for interface version > 3");
    std::exit(-1);
  }
  res.replace(res.find(TIME_INTERVAL), TIME_INTERVAL.length(), std::to_string(time_interval_));
  res.replace(res.find(SERVOJ_TIME_WAITING), SERVOJ_TIME_WAITING.length(), std::to_string(servoj_time_waiting_));
  res.replace(res.find(SERVOJ_TIME), SERVOJ_TIME.length(), std::to_string(servoj_time_));
  res.replace(res.find(MAX_WAITING_TIME), MAX_WAITING_TIME.length(), std::to_string(max_waiting_time_));
  res.replace(res.find(SERVOJ_GAIN), SERVOJ_GAIN.length(), std::to_string(servoj_gain_));
  res.replace(res.find(SERVOJ_LOOKAHEAD_TIME), SERVOJ_LOOKAHEAD_TIME.length(), std::to_string(servoj_lookahead_time_));
  res.replace(res.find(REVERSE_IP), REVERSE_IP.length(), reverse_ip);
  res.replace(res.find(REVERSE_PORT), REVERSE_PORT.length(), std::to_string(reverse_port));
  res.replace(res.find(MAX_JOINT_DIFFERENCE), MAX_JOINT_DIFFERENCE.length(), std::to_string(max_joint_difference_));
  program_ = res;

  if (!server_.bind())
  {
    LOG_ERROR("Failed to bind server, the port %d is likely already in use", reverse_port);
    std::exit(-1);
  }
  LOG_INFO("Low Bandwidth Trajectory Follower is initialized!");
}

bool LowBandwidthTrajectoryFollower::start()
{
  LOG_INFO("Starting LowBandwidthTrajectoryFollower");

  if (running_)
    return true;  // not sure

  LOG_INFO("Uploading trajectory program to robot");

  if (!commander_.uploadProg(program_))
  {
    LOG_ERROR("Program upload failed!");
    return false;
  }

  LOG_DEBUG("Awaiting incoming robot connection");

  if (!server_.accept())
  {
    LOG_ERROR("Failed to accept incoming robot connection");
    return false;
  }

  LOG_DEBUG("Robot successfully connected");
  return (running_ = true);
}

bool LowBandwidthTrajectoryFollower::execute(const std::array<double, 6> &positions,
                                             const std::array<double, 6> &velocities, double sample_number,
                                             double time_in_seconds)
{
  if (!running_)
    return false;

  std::ostringstream out;

  out << "(";
  out << sample_number << ",";
  for (auto const &pos : positions)
  {
    out << pos << ",";
  }
  for (auto const &vel : velocities)
  {
    out << vel << ",";
  }
  out << time_in_seconds << ")\r\n";

  // I know it's ugly but it's the most efficient and fastest way
  // We have only ASCII characters and we can cast char -> uint_8
  const std::string tmp = out.str();
  const char *formatted_message = tmp.c_str();
  const uint8_t *buf = (uint8_t *)formatted_message;

  size_t written;
  LOG_DEBUG("Sending message %s", formatted_message);

  return server_.write(buf, strlen(formatted_message) + 1, written);
}

bool LowBandwidthTrajectoryFollower::execute(std::vector<TrajectoryPoint> &trajectory, std::atomic<bool> &interrupt)
{
  if (!running_)
    return false;

  bool finished = false;

  char *line[MAX_SERVER_BUF_LEN];

  bool res = true;

  while (!finished && !interrupt)
  {
    if (!server_.readLine((char *)line, MAX_SERVER_BUF_LEN))
    {
      LOG_DEBUG("Connection closed. Finishing!");
      finished = true;
      break;
    }
    unsigned int message_num = atoi((const char *)line);
    LOG_DEBUG("Received request %i", message_num);
    if (message_num < trajectory.size())
    {
      res = execute(trajectory[message_num].positions, trajectory[message_num].velocities, message_num,
                    trajectory[message_num].time_from_start.count() / 1e6);
    }
    else
    {
      res = execute(EMPTY_VALUES, EMPTY_VALUES, message_num, 0.0);
    }
    if (!res)
    {
      finished = true;
    }
  }
  return res;
}

void LowBandwidthTrajectoryFollower::stop()
{
  if (!running_)
    return;

  server_.disconnectClient();
  running_ = false;
}
