// this is for emacs file handling -*- mode: c++; indent-tabs-mode: nil -*-

// -- BEGIN LICENSE BLOCK ----------------------------------------------
// -- END LICENSE BLOCK ------------------------------------------------

//----------------------------------------------------------------------
/*!\file
 *
 * \author  Felix Mauch mauch@fzi.de
 * \date    2019-04-08
 *
 */
//----------------------------------------------------------------------

#ifndef UR_RTDE_DRIVER_KINEMATICS_INFO_H_INCLUDED
#define UR_RTDE_DRIVER_KINEMATICS_INFO_H_INCLUDED

#include "ur_rtde_driver/types.h"
#include "ur_rtde_driver/primary/robot_state.h"
namespace ur_driver
{
namespace primary_interface
{
/*!
 * \brief This messages contains information about the robot's calibration. The DH parameters are
 * a combination between the perfect model parameters and the correction deltas as noted in the
 * configuration files on the robot controller.
 */
class KinematicsInfo : public RobotState
{
public:
  KinematicsInfo() = delete;
  KinematicsInfo(const RobotStateType type) : RobotState(type)
  {
  }
  virtual ~KinematicsInfo() = default;

  virtual bool parseWith(comm::BinParser& bp);
  virtual std::string toString() const;

  vector6uint32_t checksum_;
  vector6d_t dh_theta_;
  vector6d_t dh_a_;
  vector6d_t dh_d_;
  vector6d_t dh_alpha_;
  uint32_t calibration_status_;  // According to the docs this should be uint8_t, but then I have 3 bytes left.
};

// TODO: Handle pre-3.6 as they don't have kinematics info
}  // namespace primary_interface
}  // namespace ur_driver

#endif  // ifndef UR_RTDE_DRIVER_KINEMATICS_INFO_H_INCLUDED
