// this is for emacs file handling -*- mode: c++; indent-tabs-mode: nil -*-

// -- BEGIN LICENSE BLOCK ----------------------------------------------
// -- END LICENSE BLOCK ------------------------------------------------

//----------------------------------------------------------------------
/*!\file
 *
 * \author  Felix Mauch mauch@fzi.de
 * \date    2019-06-12
 *
 */
//----------------------------------------------------------------------
#ifndef CONTROLLER_STOPPER_CONTROLLER_STOPPER_H_INCLUDED
#define CONTROLLER_STOPPER_CONTROLLER_STOPPER_H_INCLUDED

#include <ros/ros.h>
#include <std_msgs/Bool.h>

class ControllerStopper
{
public:
  ControllerStopper() = delete;
  ControllerStopper(const ros::NodeHandle& nh);
  virtual ~ControllerStopper() = default;

private:
  void robotRunningCallback(const std_msgs::BoolConstPtr& msg);

  /*!
   * \brief Queries running stoppable controllers.
   *
   * Queries the controller manager for running controllers and compares the result with the
   * consistent_controllers_. The remaining running controllers are stored in stopped_controllers_
   */
  void findStoppableControllers();

  ros::NodeHandle nh_;
  ros::NodeHandle priv_nh_;
  ros::Subscriber robot_running_sub_;
  ros::ServiceClient controller_manager_srv_;
  ros::ServiceClient controller_list_srv_;

  std::vector<std::string> consistent_controllers_;
  std::vector<std::string> stopped_controllers_;

  bool robot_running_;
};
#endif  // ifndef CONTROLLER_STOPPER_CONTROLLER_STOPPER_H_INCLUDED
