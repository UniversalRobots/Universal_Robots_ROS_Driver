/*
 * Copyright 2017, 2018 Jarek Potiuk (low bandwidth trajectory follower)
 *
 * Copyright 2017, 2018 Simon Rasmussen (refactor)
 *
 * Copyright 2015, 2016 Thomas Timm Andersen (original version)
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

#pragma once

#include <actionlib/server/action_server.h>
#include <actionlib/server/server_goal_handle.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <ros/ros.h>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <mutex>
#include <set>
#include <thread>
#include "ur_modern_driver/log.h"
#include "ur_modern_driver/ros/service_stopper.h"
#include "ur_modern_driver/ros/trajectory_follower.h"
#include "ur_modern_driver/ur/consumer.h"
#include "ur_modern_driver/ur/master_board.h"
#include "ur_modern_driver/ur/state.h"

class ActionServer : public Service, public URRTPacketConsumer
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

  GoalHandle curr_gh_;
  std::atomic<bool> interrupt_traj_;
  std::atomic<bool> has_goal_, running_;
  std::mutex tj_mutex_;
  std::condition_variable tj_cv_;
  std::thread tj_thread_;

  ActionTrajectoryFollowerInterface& follower_;

  RobotState state_;
  std::array<double, 6> q_actual_, qd_actual_;

  void onGoal(GoalHandle gh);
  void onCancel(GoalHandle gh);

  bool validate(GoalHandle& gh, Result& res);
  bool validateState(GoalHandle& gh, Result& res);
  bool validateJoints(GoalHandle& gh, Result& res);
  bool validateTrajectory(GoalHandle& gh, Result& res);

  bool try_execute(GoalHandle& gh, Result& res);
  void interruptGoal(GoalHandle& gh);

  std::vector<size_t> reorderMap(std::vector<std::string> goal_joints);

  void trajectoryThread();
  bool updateState(RTShared& data);

public:
  ActionServer(ActionTrajectoryFollowerInterface& follower, std::vector<std::string>& joint_names, double max_velocity);

  void start();
  virtual void onRobotStateChange(RobotState state);

  virtual bool consume(RTState_V1_6__7& state);
  virtual bool consume(RTState_V1_8& state);
  virtual bool consume(RTState_V3_0__1& state);
  virtual bool consume(RTState_V3_2__3& state);
};
