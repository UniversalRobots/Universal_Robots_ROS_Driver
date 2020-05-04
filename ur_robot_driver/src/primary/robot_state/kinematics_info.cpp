// this is for emacs file handling -*- mode: c++; indent-tabs-mode: nil -*-

// -- BEGIN LICENSE BLOCK ----------------------------------------------
// Copyright 2019 FZI Forschungszentrum Informatik
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
// -- END LICENSE BLOCK ------------------------------------------------

//----------------------------------------------------------------------
/*!\file
 *
 * \author  Felix Exner exner@fzi.de
 * \date    2019-04-08
 *
 */
//----------------------------------------------------------------------

#include "ur_robot_driver/log.h"
#include "ur_robot_driver/primary/robot_state/kinematics_info.h"
#include "ur_robot_driver/primary/abstract_primary_consumer.h"

#include <iomanip>

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

bool KinematicsInfo ::consumeWith(AbstractPrimaryConsumer& consumer)
{
  return consumer.consume(*this);
}

std::string KinematicsInfo::toString() const
{
  std::stringstream os;
  os << "checksum: [";
  for (size_t i = 0; i < checksum_.size(); ++i)
  {
    os << checksum_[i] << " ";
  }
  os << "]" << std::endl;
  os << "dh_theta: [";
  for (size_t i = 0; i < dh_theta_.size(); ++i)
  {
    os << std::setprecision(15) << dh_theta_[i] << " ";
  }
  os << "]" << std::endl;

  os << "dh_a: [";
  for (size_t i = 0; i < dh_a_.size(); ++i)
  {
    os << std::setprecision(15) << dh_a_[i] << " ";
  }
  os << "]" << std::endl;

  os << "dh_d: [";
  for (size_t i = 0; i < dh_d_.size(); ++i)
  {
    os << std::setprecision(15) << dh_d_[i] << " ";
  }
  os << "]" << std::endl;

  os << "dh_alpha: [";
  for (size_t i = 0; i < dh_alpha_.size(); ++i)
  {
    os << std::setprecision(15) << dh_alpha_[i] << " ";
  }
  os << "]" << std::endl;

  os << "calibration_status: " << calibration_status_ << std::endl;

  return os.str();
}

std::string KinematicsInfo::toHash() const
{
  std::stringstream ss;
  for (size_t i = 0; i < 6; ++i)
  {
    ss << dh_theta_[i];
    ss << dh_d_[i];
    ss << dh_a_[i];
    ss << dh_alpha_[i];
  }
  std::hash<std::string> hash_fn;
  return "calib_" + std::to_string(hash_fn(ss.str()));
}
}  // namespace primary_interface
}  // namespace ur_driver
