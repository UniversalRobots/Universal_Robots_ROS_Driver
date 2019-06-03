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

#ifndef UR_CALIBRATION_CALIBRATION_CONSUMER_H_INCLUDED
#define UR_CALIBRATION_CALIBRATION_CONSUMER_H_INCLUDED
#include <ur_rtde_driver/comm/pipeline.h>

#include <ur_rtde_driver/primary/robot_state/kinematics_info.h>

#include <ur_calibration/calibration.h>

namespace ur_calibration
{
class CalibrationConsumer
  : public ur_driver::comm::IConsumer<ur_driver::comm::URPackage<ur_driver::primary_interface::PackageHeader>>
{
public:
  CalibrationConsumer();
  virtual ~CalibrationConsumer() = default;

  virtual void setupConsumer()
  {
  }
  virtual void teardownConsumer()
  {
  }
  virtual void stopConsumer()
  {
  }
  virtual void onTimeout()
  {
  }

  virtual bool
  consume(std::shared_ptr<ur_driver::comm::URPackage<ur_driver::primary_interface::PackageHeader>> product);

  bool isCalibrated() const
  {
    return calibrated_;
  }

  YAML::Node getCalibrationParameters() const;

private:
  bool calibrated_;
  YAML::Node calibration_parameters_;
};
}  // namespace ur_calibration
#endif  // ifndef UR_CALIBRATION_CALIBRATION_CONSUMER_H_INCLUDED
