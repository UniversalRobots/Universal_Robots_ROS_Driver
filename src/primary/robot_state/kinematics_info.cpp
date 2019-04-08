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

#include "ur_rtde_driver/log.h"
#include "ur_rtde_driver/primary/robot_state/kinematics_info.h"

namespace ur_driver
{
namespace primary_interface
{
bool KinematicsInfo::parseWith(comm::BinParser& bp)
{
  bp.parse(checksum_);
  bp.parse(dh_theta_);
  bp.parse(dh_a_);
  bp.parse(dh_d_);
  bp.parse(dh_alpha_);
  bp.parse(calibration_status_);

  return true;
}

std::string KinematicsInfo::toString() const
{
  std::stringstream os;
  os << "dh_theta: [";
  for (size_t i = 0; i < dh_theta_.size(); ++i)
  {
    os << dh_theta_[i] << " ";
  }
  os << "]" << std::endl;

  os << "dh_a: [";
  for (size_t i = 0; i < dh_a_.size(); ++i)
  {
    os << dh_a_[i] << " ";
  }
  os << "]" << std::endl;

  os << "dh_d: [";
  for (size_t i = 0; i < dh_d_.size(); ++i)
  {
    os << dh_d_[i] << " ";
  }
  os << "]" << std::endl;

  os << "dh_alpha: [";
  for (size_t i = 0; i < dh_alpha_.size(); ++i)
  {
    os << dh_alpha_[i] << " ";
  }
  os << "]" << std::endl;

  return os.str();
}
}  // namespace primary_interface
}  // namespace ur_driver
