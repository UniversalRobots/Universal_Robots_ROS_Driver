#pragma once

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <mutex>
#include <set>
#include <ros/ros.h>
#include <actionlib/server/action_server.h>
#include <actionlib/server/server_goal_handle.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include "ur_modern_driver/log.h"
#include "ur_modern_driver/ur/consumer.h"
#include "ur_modern_driver/ur/master_board.h"
#include "ur_modern_driver/ur/state.h"
#include "ur_modern_driver/ros/service_stopper.h"
#include "ur_modern_driver/ros/trajectory_follower.h"


class ActionServer : public URRTPacketConsumer, public Service
{
private:
  typedef control_msgs::FollowJointTrajectoryAction Action;
  typedef control_msgs::FollowJointTrajectoryResult Result;
  typedef actionlib::ServerGoalHandle<Action> GoalHandle;
  typedef actionlib::ActionServer<Action> Server;
  
  ros::NodeHandle nh_;
  Server as_;

  std::vector<std::string> joint_names_;
  std::set<std::string> joint_set_;
  double max_velocity_;
  RobotState state_;

  
  GoalHandle curr_gh_;
  std::atomic<bool> has_goal_, running_;
  std::mutex tj_mutex_;
  std::condition_variable tj_cv_;

  TrajectoryFollower& follower_;

  void onGoal(GoalHandle gh);
  void onCancel(GoalHandle gh);

  bool validate(GoalHandle& gh, Result& res);
  bool validateState(GoalHandle& gh, Result& res);  
  bool validateJoints(GoalHandle& gh, Result& res);
  bool validateTrajectory(GoalHandle& gh, Result& res);

  bool try_execute(GoalHandle& gh, Result& res);

  std::vector<size_t> reorderMap(std::vector<std::string> goal_joints);
  double interp_cubic(double t, double T, double p0_pos, double p1_pos, double p0_vel, double p1_vel);

  void trajectoryThread();

  template <typename U>
  double toSec(U const& u)
  {
      return std::chrono::duration_cast<std::chrono::duration<double>>(u).count();
  }

public:
  ActionServer(TrajectoryFollower& follower, std::vector<std::string>& joint_names, double max_velocity);

  virtual void onRobotStateChange(RobotState state);
};