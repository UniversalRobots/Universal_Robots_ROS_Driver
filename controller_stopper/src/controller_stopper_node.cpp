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

#include <ros/ros.h>

#include <controller_stopper/controller_stopper.h>

int main(int argc, char** argv)
{
  // Set up ROS.
  ros::init(argc, argv, "controller_stopper_node");
  ros::NodeHandle nh;
  ros::NodeHandle priv_nh("");

  ControllerStopper stopper(nh);

  ros::spin();
  return 0;
}
