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
 * \date    2019-04-17
 *
 */
//----------------------------------------------------------------------

#ifndef UR_CONTROLLERS_SPEED_SCALING_STATE_CONTROLLER_H_INCLUDED
#define UR_CONTROLLERS_SPEED_SCALING_STATE_CONTROLLER_H_INCLUDED

#include <controller_interface/controller.h>
#include <realtime_tools/realtime_publisher.h>
#include <std_msgs/Float64.h>

#include "ur_controllers/speed_scaling_interface.h"

namespace ur_controllers
{
class SpeedScalingStateController : public controller_interface::Controller<SpeedScalingInterface>
{
public:
  SpeedScalingStateController() = default;
  virtual ~SpeedScalingStateController() override = default;

  virtual bool init(SpeedScalingInterface* hw, ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh) override;
  virtual void starting(const ros::Time& time) override;
  virtual void update(const ros::Time& time, const ros::Duration& /*period*/) override;
  virtual void stopping(const ros::Time& /*time*/) override;

private:
  std::vector<SpeedScalingHandle> sensors_;
  // TODO: We should use a custom message for this
  typedef std::shared_ptr<realtime_tools::RealtimePublisher<std_msgs::Float64> > RtPublisherPtr;
  std::vector<RtPublisherPtr> realtime_pubs_;
  std::vector<ros::Time> last_publish_times_;
  double publish_rate_;
};
}  // namespace ur_controllers
#endif  // ifndef UR_CONTROLLERS_SPEED_SCALING_STATE_CONTROLLER_H_INCLUDED
