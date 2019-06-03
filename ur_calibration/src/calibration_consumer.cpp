// this is for emacs file handling -*- mode: c++; indent-tabs-mode: nil -*-

// -- BEGIN LICENSE BLOCK ----------------------------------------------
// -- END LICENSE BLOCK ------------------------------------------------

//----------------------------------------------------------------------
/*!\file
 *
 * \author  Felix Mauch mauch@fzi.de
 * \date    2019-05-28
 *
 */
//----------------------------------------------------------------------

#include <ur_calibration/calibration_consumer.h>

namespace ur_calibration
{
CalibrationConsumer::CalibrationConsumer() : calibrated_(false)
{
}

bool CalibrationConsumer::consume(
    std::shared_ptr<ur_driver::comm::URPackage<ur_driver::primary_interface::PackageHeader>> product)
{
  auto kin_info = std::dynamic_pointer_cast<ur_driver::primary_interface::KinematicsInfo>(product);
  if (kin_info != nullptr)
  {
    LOG_INFO("%s", product->toString().c_str());
    DHRobot my_robot;
    for (size_t i = 0; i < kin_info->dh_a_.size(); ++i)
    {
      my_robot.segments_.push_back(
          DHSegment(kin_info->dh_d_[i], kin_info->dh_a_[i], kin_info->dh_theta_[i], kin_info->dh_alpha_[i]));
    }
    Calibration calibration(my_robot);
    calibration.correctChain();

    calibration_parameters_ = calibration.toYaml();
    calibrated_ = true;
  }
  return true;
}

YAML::Node CalibrationConsumer::getCalibrationParameters() const
{
  if (!calibrated_)
  {
    throw(std::runtime_error("Cannot get calibration, as no calibration data received yet"));
  }
  return calibration_parameters_;
}
}  // namespace ur_calibration
