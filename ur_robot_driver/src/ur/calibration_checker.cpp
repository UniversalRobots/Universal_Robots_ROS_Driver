// this is for emacs file handling -*- mode: c++; indent-tabs-mode: nil -*-

// -- BEGIN LICENSE BLOCK ----------------------------------------------
// -- END LICENSE BLOCK ------------------------------------------------

//----------------------------------------------------------------------
/*!\file
 *
 * \author  Felix Exner exner@fzi.de
 * \date    2019-06-14
 *
 */
//----------------------------------------------------------------------

#include <ur_robot_driver/ur/calibration_checker.h>

namespace ur_driver
{
CalibrationChecker::CalibrationChecker(const std::string& expected_hash)
  : expected_hash_(expected_hash), checked_(false)
{
}
void CalibrationChecker::handle(primary_interface::KinematicsInfo& kin_info)
{
  if (kin_info.toHash() != expected_hash_)
  {
    LOG_ERROR("The calibration parameters of the connected robot don't match the ones from the given kinematics "
              "config file. Please be aware that this can lead to critical inaccuracies of tcp positions. Use the "
              "ur_calibration tool to extract the correct calibration from the robot and pass that into the "
              "description. See "
              "[https://github.com/UniversalRobots/Universal_Robots_ROS_Driver#extract-calibration-information] for "
              "details.");
  }
  else
  {
    LOG_INFO("Calibration checked successfully.");
  }

  checked_ = true;
}

}  // namespace ur_driver
