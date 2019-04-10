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
 * \author  Tristan Schnell schnell@fzi.de
 * \date    2019-04-10
 *
 */
//----------------------------------------------------------------------

#include "ur_rtde_driver/rtde/rtde_client.h"

namespace ur_driver
{
namespace rtde_interface
{
RTDEClient::RTDEClient(std::string ROBOT_IP, comm::INotifier& notifier)
    : stream_(ROBOT_IP, UR_RTDE_PORT), parser_(), prod_(stream_, parser_),
      pipeline_(prod_, PIPELINE_NAME, notifier)
{
  pipeline_.run();

  sleep(1);

  uint8_t buffer[4096];
  size_t size;
  size_t written;
  size = RequestProtocolVersionRequest::generateSerializedRequest(buffer);
  std::cout << "size: " << size << std::endl;
  std::cout << "buffer: " << static_cast<int>(buffer[0]) << " - "
            << static_cast<int>(buffer[1]) << " - "
            << static_cast<int>(buffer[2]) << " - "
            << static_cast<int>(buffer[3]) << " - "
            << static_cast<int>(buffer[4]) << std::endl;
   stream_.write(buffer, size, written);

  std::cout << "wrote: " << written << std::endl;
}

bool RTDEClient::getDataPackage(
    std::unique_ptr<comm::URPackage<PackageHeader>>& data_package,
    std::chrono::milliseconds timeout)
{
  return pipeline_.getLatestProduct(data_package, timeout);
}
} // namespace rtde_interface
} // namespace ur_driver
