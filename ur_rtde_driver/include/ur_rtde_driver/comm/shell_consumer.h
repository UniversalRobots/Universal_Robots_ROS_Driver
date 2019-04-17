// this is for emacs file handling -*- mode: c++; indent-tabs-mode: nil -*-

// -- BEGIN LICENSE BLOCK ----------------------------------------------
// -- END LICENSE BLOCK ------------------------------------------------

//----------------------------------------------------------------------
/*!\file
 *
 * \author  Felix Mauch mauch@fzi.de
 * \date    2019-04-09
 *
 */
//----------------------------------------------------------------------

#ifndef UR_RTDE_DRIVER_SHELL_CONSUMER_H_INCLUDED
#define UR_RTDE_DRIVER_SHELL_CONSUMER_H_INCLUDED

#include "ur_rtde_driver/log.h"
#include "ur_rtde_driver/comm/pipeline.h"
#include "ur_rtde_driver/comm/package.h"

namespace ur_driver
{
namespace comm
{
template <typename HeaderT>
class ShellConsumer : public IConsumer<URPackage<HeaderT>>
{
public:
  ShellConsumer() = default;
  virtual ~ShellConsumer() = default;

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

  virtual bool consume(std::shared_ptr<URPackage<HeaderT>> product)
  {
    LOG_INFO("%s", product->toString().c_str());
    return true;
  }

private:
  /* data */
};
}  // namespace comm
}  // namespace ur_driver
#endif  // ifndef UR_RTDE_DRIVER_SHELL_CONSUMER_H_INCLUDED
