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

#ifndef UR_RTDE_DRIVER_RTDE_CLIENT_H_INCLUDED
#define UR_RTDE_DRIVER_RTDE_CLIENT_H_INCLUDED

#include "ur_rtde_driver/comm/pipeline.h"
#include "ur_rtde_driver/rtde/package_header.h"
#include "ur_rtde_driver/rtde/rtde_package.h"
#include "ur_rtde_driver/comm/stream.h"
#include "ur_rtde_driver/rtde/rtde_parser.h"
#include "ur_rtde_driver/comm/producer.h"
#include "ur_rtde_driver/rtde/data_package.h"
#include "ur_rtde_driver/rtde/request_protocol_version.h"
#include "ur_rtde_driver/rtde/control_package_setup_outputs.h"
#include "ur_rtde_driver/rtde/control_package_start.h"
#include "ur_rtde_driver/log.h"
#include "ur_rtde_driver/rtde/rtde_writer.h"

static const int UR_RTDE_PORT = 30004;
static const std::string PIPELINE_NAME = "RTDE Data Pipeline";

namespace ur_driver
{
namespace rtde_interface
{
class RTDEClient
{
public:
  RTDEClient() = delete;
  RTDEClient(std::string robot_ip, comm::INotifier& notifier, const std::string& output_recipe_file,
             const std::string& input_recipe_file);
  ~RTDEClient() = default;
  bool init();
  bool start();
  bool getDataPackage(std::unique_ptr<comm::URPackage<PackageHeader>>& data_package, std::chrono::milliseconds timeout);

  double getMaxFrequency() const
  {
    return max_frequency_;
  }

  VersionInformation getVersion()
  {
    return urcontrol_version_;
  }

  /*!
   * \brief Returns the IP address (of the machine running this driver) used for the socket connection.
   *
   * \returns The IP address as a string (e.g. "192.168.0.1")
   */
  std::string getIP() const;

  RTDEWriter& getWriter();

private:
  comm::URStream<PackageHeader> stream_;
  std::vector<std::string> recipe_;
  std::vector<std::string> input_recipe_;
  RTDEParser parser_;
  comm::URProducer<PackageHeader> prod_;
  comm::Pipeline<PackageHeader> pipeline_;
  RTDEWriter writer_;

  VersionInformation urcontrol_version_;

  double max_frequency_;

  constexpr static const double CB3_MAX_FREQUENCY = 125.0;
  constexpr static const double URE_MAX_FREQUENCY = 500.0;

  std::vector<std::string> readRecipe(const std::string& recipe_file);
};

}  // namespace rtde_interface
}  // namespace ur_driver

#endif  // UR_RTDE_DRIVER_RTDE_CLIENT_H_INCLUDED
