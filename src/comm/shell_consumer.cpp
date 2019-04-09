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

#include "ur_rtde_driver/comm/shell_consumer.h"

namespace ur_driver
{
namespace comm
{
template <typename HeaderT>
bool ShellConsumer<HeaderT>::consume(std::shared_ptr<URPackage<HeaderT>> pkg)
{
  LOG_INFO("%s", pkg->toString());
  return true;
}
}  // namespace comm

}  // namespace ur_driver
