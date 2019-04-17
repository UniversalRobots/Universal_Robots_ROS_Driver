// this is for emacs file handling -*- mode: c++; indent-tabs-mode: nil -*-

// -- BEGIN LICENSE BLOCK ----------------------------------------------
// -- END LICENSE BLOCK ------------------------------------------------

//----------------------------------------------------------------------
/*!\file
 *
 * \author  Felix Mauch mauch@fzi.de
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
  //TODO: We should use a better datatype later on
  typedef std::shared_ptr<realtime_tools::RealtimePublisher<std_msgs::Float64> > RtPublisherPtr;
  std::vector<RtPublisherPtr> realtime_pubs_;
  std::vector<ros::Time> last_publish_times_;
  double publish_rate_;
};
}  // namespace ur_controllers
#endif  // ifndef UR_CONTROLLERS_SPEED_SCALING_STATE_CONTROLLER_H_INCLUDED
