#include <cmath>
#include "ur_modern_driver/ros/action_server.h"

ActionServer::ActionServer(TrajectoryFollower& follower, std::vector<std::string>& joint_names, double max_velocity)
  : as_(
      nh_, 
      "follow_joint_trajectory",
      boost::bind(&ActionServer::onGoal, this, _1),
      boost::bind(&ActionServer::onCancel, this, _1), 
      false
    )
  , joint_names_(joint_names)
  , joint_set_(joint_names.begin(), joint_names.end())
  , max_velocity_(max_velocity)
  , state_(RobotState::Error)
  , follower_(follower)
{
  
}

void ActionServer::onRobotStateChange(RobotState state)
{
  state_ = state;
}

void ActionServer::onGoal(GoalHandle gh)
{
  Result res;
  res.error_code = -100;

  if(!validate(gh, res) || !try_execute(gh, res))
    gh.setRejected(res, res.error_string);
}

void ActionServer::onCancel(GoalHandle gh)
{

}

bool ActionServer::validate(GoalHandle& gh, Result& res)
{ 
  return !validateState(gh, res) || !validateJoints(gh, res) || !validateTrajectory(gh, res);
}

bool ActionServer::validateState(GoalHandle& gh, Result& res)
{
  switch(state_)
  {
    case RobotState::EmergencyStopped:
      res.error_string = "Robot is emergency stopped";
      return false;
      
    case RobotState::ProtectiveStopped:
      res.error_string = "Robot is protective stopped";
      return false;
      
    case RobotState::Error:
      res.error_string = "Robot is not ready, check robot_mode";
      return false;

    case RobotState::Running:
      return true;

    default:
      res.error_string = "Undefined state";
      return false;
  }
}

bool ActionServer::validateJoints(GoalHandle& gh, Result& res)
{
  auto goal = gh.getGoal();
  auto const& joints = goal->trajectory.joint_names;
  std::set<std::string> goal_joints(joints.begin(), joints.end());

  if(goal_joints == joint_set_)
    return true;

  res.error_code = Result::INVALID_JOINTS;
  res.error_string = "Invalid joint names for goal";
  return false;
}

bool ActionServer::validateTrajectory(GoalHandle& gh, Result& res)
{
  auto goal = gh.getGoal();
  res.error_code = Result::INVALID_GOAL;

  for(auto const& point : goal->trajectory.points)
  {
    if(point.velocities.size() != joint_names_.size())
    {
      res.error_code = Result::INVALID_GOAL;
      res.error_string = "Received a goal with an invalid number of velocities";
      return false;
    }
    
    if(point.positions.size() != joint_names_.size())
    {
      res.error_code = Result::INVALID_GOAL;
      res.error_string = "Received a goal with an invalid number of positions";
      return false;
    }
    
    for(auto const& velocity : point.velocities)
    {
      if(!std::isfinite(velocity))
      {
        res.error_string = "Received a goal with infinities or NaNs in velocity";
        return false;
      }
      if(std::fabs(velocity) > max_velocity_)
      {
        res.error_string = "Received a goal with velocities that are higher than " + std::to_string(max_velocity_);
        return false;
      }
    }
    for(auto const& position : point.positions)
    {
      if(!std::isfinite(position))
      {
        res.error_string = "Received a goal with infinities or NaNs in positions";
        return false;
      }
    }
  }

  return true;
}

bool ActionServer::try_execute(GoalHandle& gh, Result& res)
{
  if(!running_)
  {
    res.error_string = "Internal error";
    return false;
  }
  if(!tj_mutex_.try_lock())
  {
    has_goal_ = false;
    //stop_trajectory();
    res.error_string = "Received another trajectory";
    curr_gh_.setAborted(res, res.error_string);
    tj_mutex_.lock();
  }
  //locked here
  curr_gh_ = gh;
  has_goal_ = true;
  tj_mutex_.unlock();
  tj_cv_.notify_one();
}

inline double ActionServer::interp_cubic(double t, double T, double p0_pos, double p1_pos, double p0_vel, double p1_vel)
{
  using std::pow;
  double a = p0_pos;
  double b = p0_vel;
  double c = (-3 * a + 3 * p1_pos - 2 * T * b - T * p1_vel) / pow(T, 2);
  double d = (2 * a - 2 * p1_pos + T * b + T * p1_vel) / pow(T, 3);
  return a + b * t + c * pow(t, 2) + d * pow(t, 3);
}

std::vector<size_t> ActionServer::reorderMap(std::vector<std::string> goal_joints)
{
  std::vector<size_t> indecies;
  for(auto const& aj : joint_names_)
  {
    size_t j = 0;
    for(auto const& gj : goal_joints)
    {
      if(aj == gj)
        break;
      j++;
    }
    indecies.push_back(j);
  }
  return indecies;
}

void ActionServer::trajectoryThread()
{
  while(running_)
  {
    std::unique_lock<std::mutex> lk(tj_mutex_);
    if(!tj_cv_.wait_for(lk, std::chrono::milliseconds(100), [&]{return running_ && has_goal_;}))
      continue;
    
    auto g = curr_gh_.getGoal();
    auto const& traj = g->trajectory;
    auto const& points = traj.points;
    size_t len = points.size();
    auto const& last_point = points[points.size() - 1];
    double end_time = last_point.time_from_start.toSec();
    
    auto mapping = reorderMap(traj.joint_names);
    std::chrono::high_resolution_clock::time_point t0, t;
    t = t0 = std::chrono::high_resolution_clock::now();

    size_t i = 0;
    while(end_time >= toSec(t - t0) && has_goal_)
    {
      while(points[i].time_from_start.toSec() <= toSec(t - t0) && i < len)
        i++;
      
      auto const& pp = points[i-1];
      auto const& p = points[i];

      auto pp_t = pp.time_from_start.toSec();
      auto p_t =p.time_from_start.toSec();

      std::array<double, 6> pos;
      for(size_t j = 0; j < pos.size(); j++)
      {
        pos[i] = interp_cubic(
          toSec(t - t0) - pp_t,
          p_t - pp_t,
          pp.positions[j],
          p.positions[j],
          pp.velocities[j],
          p.velocities[j]
        );
      }

      follower_.execute(pos);
      //std::this_thread::sleep_for(std::chrono::milliseconds((int)((servoj_time_ * 1000) / 4.))); 
      t = std::chrono::high_resolution_clock::now();
    }

    has_goal_ = false;
  }
}