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

void ActionServer::start()
{
  if(running_)
    return;
  running_ = true;
  tj_thread_ = thread(&ActionServer::trajectoryThread, this);
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
  interrupt_traj_ = true;
  //wait for goal to be interrupted
  std::lock_guard<std::mutex> lock(tj_mutex_);

  Result res;
  res.error_code = -100;
  res.error_string = "Goal cancelled by client";
  gh.setCanceled(res);
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

  //todo validate start position?

  return true;
}

inline std::chrono::microseconds convert(const ros::Duration &dur)
{
  return std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::seconds(dur.sec)) 
    + std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::nanoseconds(dur.nsec));
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
    interrupt_traj_ = true;
    res.error_string = "Received another trajectory";
    curr_gh_.setAborted(res, res.error_string);
    tj_mutex_.lock();
    //todo: make configurable
    std::this_thread::sleep_for(std::chrono::milliseconds(250));
  }
  //locked here
  curr_gh_ = gh;
  interrupt_traj_ = false;
  has_goal_ = true;
  tj_mutex_.unlock();
  tj_cv_.notify_one();
  return true;
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
  follower_.start(); //todo check error
  //as_.start();
  while(running_)
  {
    std::unique_lock<std::mutex> lk(tj_mutex_);
    if(!tj_cv_.wait_for(lk, std::chrono::milliseconds(100), [&]{return running_ && has_goal_;}))
      continue;
    
    LOG_DEBUG("Trajectory received and accepted");
    curr_gh_.setAccepted();

    auto goal = curr_gh_.getGoal();
    auto mapping = reorderMap(goal->trajectory.joint_names);
    std::vector<TrajectoryPoint> trajectory(goal->trajectory.points.size());
    
    for(auto const& point : goal->trajectory.points)
    {
      std::array<double, 6> pos, vel;
      for(size_t i = 0; i < 6; i++)
      {
        //joint names of the goal might have a different ordering compared
        //to what URScript expects so need to map between the two
        size_t idx = mapping[i];
        pos[idx] = point.positions[i];
        vel[idx] = point.velocities[i];
      }
      trajectory.push_back(TrajectoryPoint(pos, vel, convert(point.time_from_start)));
    }

    Result res;
    if(follower_.execute(trajectory, interrupt_traj_))
    {
      //interrupted goals must be handled by interrupt trigger
      if(!interrupt_traj_)
      {
        LOG_DEBUG("Trajectory executed successfully");
        res.error_code = Result::SUCCESSFUL;
        curr_gh_.setSucceeded(res);
      }
    }
    else
    {
      LOG_DEBUG("Trajectory failed");
      res.error_code = -100;
      res.error_string = "Connection to robot was lost";      
      curr_gh_.setAborted(res, res.error_string);
    }

    has_goal_ = false;
    lk.unlock();
  }
  follower_.stop();
}