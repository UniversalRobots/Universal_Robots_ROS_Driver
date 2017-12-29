#include "ur_modern_driver/ros/safe_trajectory_follower.h"
#include <endian.h>
#include <cmath>
#include <ros/ros.h>

static const std::array<double, 6> EMPTY_VALUES = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

static const std::string TIME_INTERVAL("{{TIME_INTERVAL}}");
static const std::string SERVOJ_TIME("{{SERVOJ_TIME}}");
static const std::string SERVOJ_TIME_WAITING("{{SERVOJ_TIME_WAITING}}");
static const std::string MAX_WAITING_TIME("{{MAX_WAITING_TIME}}");
static const std::string DEBUG("{{DEBUG}}");
static const std::string MORE_DEBUG("{{MORE_DEBUG}}");
static const std::string SERVOJ_GAIN("{{SERVOJ_GAIN}}");
static const std::string SERVOJ_LOOKAHEAD_TIME("{{SERVOJ_LOOKAHEAD_TIME}}");
static const std::string REVERSE_IP("{{REVERSE_IP}}");
static const std::string REVERSE_PORT("{{REVERSE_PORT}}");
static const std::string POSITION_PROGRAM = R"(
def driveRobotSafeTrajectory():

    global JOINT_NUM               = 6
    global TIME_INTERVAL           = {{TIME_INTERVAL}}
    global SERVOJ_TIME             = {{SERVOJ_TIME}}
    global SERVOJ_TIME_WAITING     = {{SERVOJ_TIME_WAITING}}
    global MAX_WAITING_TIME        = {{MAX_WAITING_TIME}}
    global DEBUG                   = {{DEBUG}}
    global MORE_DEBUG              = {{MORE_DEBUG}}
    global EMPTY_VALUES            = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    global SERVOJ_GAIN             = {{SERVOJ_GAIN}}
    global SERVOJ_LOOKAHEAD_TIME   = {{SERVOJ_LOOKAHEAD_TIME}}
    global CONNECTION_NAME         = "reverse_connection"
    global REVERSE_IP              = "{{REVERSE_IP}}"
    global REVERSE_PORT            = {{REVERSE_PORT}}

    # NOTE All the global variables here are accessed by different threads
    # therefore they should be accessed within critical section. Those variables
    # are all prefixed with g_ . Whenever their values are needed they are copied
    # to similarly named l_ variable. Copying happens inside the critical section
    # and l_values might be used outside of it. This needs to be confirmed with UR
    # about the semantics of assignment operator (copying or by reference?).
    # Hopefully it's copying :).
    #
    # Please make sure to keep that pattern and do not access the global variables
    # outside of the critical section
    #
    # TO DO: Are those assignments by references or copies? We will find out soon
    #        If not then assigning positions would not work either
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
        while l_previous_index < 1 + JOINT_NUM * 2 + 2: # Note - we do not check first two which are non-zero
            if waypoint[l_previous_index] != 0.0:
                return False
            end
            l_previous_index = l_previous_index + 1
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

        # Rotate the values received so far:
        # target -> previous
        g_position_previous = g_position_target
        g_velocity_previous = g_velocity_target
        g_time_previous     = g_time_target
        g_num_previous      = g_num_target

        # next -> previous
        g_position_target   = g_position_next
        g_velocity_target   = g_velocity_next
        g_time_target       = g_time_next
        g_num_target        = g_num_next

        # Decode the array received into next
        # waypoint 0 is number of entries in the received array
        g_num_next = waypoint[1]
        g_position_next     = [waypoint[2], waypoint[3], waypoint[4], waypoint[5], waypoint[6], waypoint[7]]
        g_velocity_next     = [waypoint[8], waypoint[9], waypoint[10], waypoint[11], waypoint[12], waypoint[13]]
        g_time_next         = waypoint[14]

        # store latest received waypoint number so that controlling thread knows it's been received already
        g_received_waypoints_number = g_num_next

        if DEBUG:
            local l_received_waypoints_number  = g_received_waypoints_number
            local l_waypoint = waypoint
        end

        exit_critical

        if DEBUG:
            textmsg("Received waypoint:")
            textmsg(l_received_waypoints_number)
            textmsg(l_waypoint)
        end
    end

    # Thread controlling the motor. In the loop it checks first if it received the
    # requested waypoints and until it does, it syncs doing noting to the motor
    # once it received all up to requested waypoints it executes interpolation
    # between PREVIOUS AND TARGET points received and rquests the next waypoint request
    # to be sent.
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
            l_received_waypoints_number = g_received_waypoints_number
            exit_critical

            local l_max_waiting_time_left = MAX_WAITING_TIME
            # if expected waypoint number not yet received wait so that receiving thread has time to receive it
            while l_received_waypoints_number < l_requested_waypoints_number and l_max_waiting_time_left > 0:

                if DEBUG:
                    textmsg("Waiting for the received waypoints number to catch up (received/requested):")
                    textmsg(l_received_waypoints_number)
                    textmsg(l_requested_waypoints_number)
                end
                # Keep robot in l_current position for short time and check if the next waipoint arrived
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

            # OK. We received next point, copy the required global variables into local ones
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

            # Note we deliberately only read "stopping" state here and not update it below
            # so that stopping flag takes effect only after one additional interpolation loop is complete
            # and all points of the trajectory are processeed
            local l_stopping_after_next_interpolation = g_stopping

            # And increasing the global requested number - informs sender thread that it needs to ask for it
            g_requested_waypoints_number = g_requested_waypoints_number + 1
            exit_critical

            if DEBUG:
                textmsg("Starting interpolation. Segment from/to:")
                textmsg(l_start_num)
                textmsg(l_end_num)
                textmsg("Current time:")
                textmsg(l_total_elapsed_time)
                textmsg("Starting/Ending segment time:")
                textmsg(l_start_time)
                textmsg(l_end_time)
            end

            l_current_position = l_start_pos

            local l_total_segment_time = l_end_time - l_start_time

            # Here perform the interpolation loop
            while l_total_elapsed_time <= l_end_time:
                if MORE_DEBUG:
                    textmsg("Next step of interpolation:")
                end

                local l_segment_elapsed_time = l_total_elapsed_time - l_start_time

                # Calculate interpolation for all joints
                j = 0
                while j < JOINT_NUM:
                    l_current_position[j] = interpolate(l_segment_elapsed_time, l_total_segment_time, l_start_pos[j], l_end_pos[j], l_start_vel[j], l_end_vel[j])
                    j = j + 1
                end

                if MORE_DEBUG:
                    textmsg("Next step of interpolated position:")
                    textmsg(l_current_position)
                    textmsg("Current time:")
                    textmsg(l_total_elapsed_time)
                    textmsg("Running servoj command:")
                end

                servoj(l_current_position,t=SERVOJ_TIME,lookahead_time=SERVOJ_LOOKAHEAD_TIME,gain=SERVOJ_GAIN)

                enter_critical
                g_total_elapsed_time = g_total_elapsed_time + TIME_INTERVAL
                l_total_elapsed_time = g_total_elapsed_time
                exit_critical

                if MORE_DEBUG:
                    textmsg("Finishing interpolation step at time:")
                    textmsg(l_total_elapsed_time)
                end
            end
            if DEBUG:
                textmsg("Ending interpolation segment at time:")
                textmsg(l_total_elapsed_time)
            end
            if l_stopping_after_next_interpolation:
                textmsg("Stopping the controlling thread on signal from receiving thread after one interpolation loop.")
                break
            end
        end
        textmsg("Ending controlling thread")
    end

    # This thread sends requested waypoints number to the client when requested number is changed
    # It will send all the numbers from [already sent + 1, g_requested_waypoints_number] and waits
    # until requested waypoints number increases
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
            # wait until we have more requested waypoints to send than actually sent ones
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
            if DEBUG:
                textmsg("Sent waypoint request number:")
                textmsg(l_sent_waypoints_number)
            end
        end
        textmsg("Joining controlling thread")
        join controlling_thread
        textmsg("Ending Sending thread")
    end

    # Receiving thread  - it will receive the next trajectory point over the TCP connection
    # It will increase the received waipoints_number on each request.
    thread receivingThread():
        local sending_thread = run sendingThread()
        while True:
            waypoint_received = socket_read_ascii_float(14, CONNECTION_NAME)
            if waypoint_received[0] == 0:
                # No new waypoint requested for the last 2 seconds
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
                textmsg("Received sentinel waypoint. Finishing.")

                enter_critical
                g_stopping = True
                g_received_waypoints_number = g_received_waypoints_number + 1
                exit_critical

                break
            end
            add_next_waypoint(waypoint_received)
        end
        textmsg("Joining sendingThread")
        join sending_thread
        textmsg("Ending Receiving thread")
    end

    textmsg("Opening socket")
    socket_open(REVERSE_IP, REVERSE_PORT, CONNECTION_NAME)
    textmsg("Socket opened")

    receiving_thread = run receivingThread()
    textmsg("Joining receiving_thread")
    join receiving_thread
    textmsg("Closing reverse connection")
    socket_close(CONNECTION_NAME)
    textmsg("Exiting the program")
end

)";

SafeTrajectoryFollower::SafeTrajectoryFollower(URCommander &commander, std::string &reverse_ip, int reverse_port,
                                       bool version_3)
  : running_(false)
  , commander_(commander)
  , server_(reverse_port)
  , time_interval_(0.008)
  , servoj_time_(0.008)
  , servoj_time_waiting_(0.001)
  , max_waiting_time_(2.0)
  , servoj_gain_(300.0)
  , servoj_lookahead_time_(0.03)
  , debug_(false)
  , more_debug_(false)
{
  ros::param::get("~time_interval", time_interval_);
  ros::param::get("~servoj_time", servoj_time_);
  ros::param::get("~servoj_time_waiting", servoj_time_waiting_);
  ros::param::get("~max_waiting_time", max_waiting_time_);
  ros::param::get("~servoj_gain", servoj_gain_);
  ros::param::get("~servoj_lookahead_time", servoj_lookahead_time_);
  ros::param::get("~debug", debug_);
  ros::param::get("~more_debug", more_debug_);

  std::string res(POSITION_PROGRAM);
  std::ostringstream out;
  if (!version_3) {
    LOG_ERROR("Safe Trajectory Follower only works for interface version > 3");
    std::exit(-1);
  }
  res.replace(res.find(TIME_INTERVAL), TIME_INTERVAL.length(), std::to_string(time_interval_));
  res.replace(res.find(SERVOJ_TIME_WAITING), SERVOJ_TIME_WAITING.length(), std::to_string(servoj_time_waiting_));
  res.replace(res.find(SERVOJ_TIME), SERVOJ_TIME.length(), std::to_string(servoj_time_));
  res.replace(res.find(MAX_WAITING_TIME), MAX_WAITING_TIME.length(), std::to_string(max_waiting_time_));
  res.replace(res.find(SERVOJ_GAIN), SERVOJ_GAIN.length(), std::to_string(servoj_gain_));
  res.replace(res.find(SERVOJ_LOOKAHEAD_TIME), SERVOJ_LOOKAHEAD_TIME.length(), std::to_string(servoj_lookahead_time_));
  res.replace(res.find(DEBUG), DEBUG.length(), debug_ ? "True" : "False");
  res.replace(res.find(MORE_DEBUG), MORE_DEBUG.length(), more_debug_ ? "True" : "False");
  res.replace(res.find(REVERSE_IP), REVERSE_IP.length(), reverse_ip);
  res.replace(res.find(REVERSE_PORT), REVERSE_PORT.length(), std::to_string(reverse_port));
  program_ = res;

  if (!server_.bind())
  {
    LOG_ERROR("Failed to bind server, the port %d is likely already in use", reverse_port);
    std::exit(-1);
  }
  LOG_INFO("Safe Trajectory Follower is initialized!");
}

bool SafeTrajectoryFollower::start()
{
  LOG_INFO("Starting SafeTrajectoryFollower");

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

bool SafeTrajectoryFollower::execute(const std::array<double, 6> &positions,
                                     const std::array<double, 6> &velocities,
                                     double sample_number, double time_in_seconds)
{
  if (!running_)
    return false;

  std::ostringstream out;

  out << "(";
  out << sample_number << ",";
  for (auto const &pos: positions)
  {
    out << pos << ",";
  }
  for (auto const &vel: velocities)
  {
    out << vel << ",";
  }
  out << time_in_seconds << ")\r\n";

  // I know it's ugly but it's the most efficient and fastest way
  // We have only ASCII characters and we can cast char -> uint_8
  const std::string tmp = out.str();
  const char *formatted_message = tmp.c_str();
  const uint8_t *buf = (uint8_t *) formatted_message;

  size_t written;
  LOG_DEBUG("Sending message %s", formatted_message);

  return server_.write(buf, strlen(formatted_message) + 1, written);
}

bool SafeTrajectoryFollower::execute(std::vector<TrajectoryPoint> &trajectory, std::atomic<bool> &interrupt)
{
  if (!running_)
    return false;

  bool finished = false;

  char* line[MAX_SERVER_BUF_LEN];

  bool res = true;

  while (!finished && !interrupt)
  {
     if (!server_.readLine((char *)line, MAX_SERVER_BUF_LEN))
     {
        LOG_DEBUG("Connection closed. Finishing!");
        finished = true;
        break;
     }
     unsigned int message_num=atoi((const char *) line);
     LOG_DEBUG("Received request %i", message_num);
     if (message_num < trajectory.size())
     {
        res = execute(trajectory[message_num].positions, trajectory[message_num].velocities,
                message_num, trajectory[message_num].time_from_start.count() / 1e6);
     } else
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

void SafeTrajectoryFollower::stop()
{
  if (!running_)
    return;

  server_.disconnectClient();
  running_ = false;
}
