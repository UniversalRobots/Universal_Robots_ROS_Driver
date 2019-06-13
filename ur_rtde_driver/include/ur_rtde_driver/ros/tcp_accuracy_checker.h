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
 * \author  Felix Mauch mauch@fzi.de
 * \date    2019-06-13
 *
 */
//----------------------------------------------------------------------

#ifndef UR_RTDE_DRIVER_ROS_TCP_ACCURACY_CHECKER_H_INCLUDED
#define UR_RTDE_DRIVER_ROS_TCP_ACCURACY_CHECKER_H_INCLUDED

#include <tf/transform_listener.h>
#include <thread>

namespace ur_driver
{
class TcpAccuracyChecker
{
public:
  TcpAccuracyChecker() = delete;
  TcpAccuracyChecker(const std::string& frame_a, const std::string& frame_b, const double desired_accuracy);
  virtual ~TcpAccuracyChecker() = default;

  /*!
   * \brief Performs a lookup between the configured frames and checks if the distance is smaller
   * than the given tolerance.
   *
   * \returns True of accuracy is below the tolerance
   */
  bool checkAccuracy();

  void asyncCheck(const uint32_t interval, const uint32_t num_checks);

private:
  tf::TransformListener tf_listener_;
  tf::StampedTransform transform_;
  std::string frame_a_;
  std::string frame_b_;
  std::unique_ptr<std::thread> worker_thread_;
  double desired_accuracy_;
  double actual_accuracy_;
};
}  // namespace ur_driver
#endif  // ifndef UR_RTDE_DRIVER_ROS_TCP_ACCURACY_CHECKER_H_INCLUDED
