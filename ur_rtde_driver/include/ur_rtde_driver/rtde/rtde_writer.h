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
 * \date    2019-07-25
 *
 */
//----------------------------------------------------------------------

#ifndef UR_RTDE_DRIVER_RTDE_WRITER_H_INCLUDED
#define UR_RTDE_DRIVER_RTDE_WRITER_H_INCLUDED

#include "ur_rtde_driver/rtde/package_header.h"
#include "ur_rtde_driver/rtde/rtde_package.h"
#include "ur_rtde_driver/comm/stream.h"

namespace ur_driver
{
namespace rtde_interface
{
class RTDEWriter
{
public:
  RTDEWriter() = delete;
  RTDEWriter(comm::URStream<PackageHeader>* stream, const std::string& recipe_file);
  ~RTDEWriter() = default;
  bool init(uint8_t recipe_id);
  bool start();

  bool sendSpeedSlider(double speed_slider_fraction);
  bool sendStandardDigitalOutput(uint8_t output_pin, bool value);
  bool sendConfigurableDigitalOutput(uint8_t output_pin, bool value);
  bool sendToolDigitalOutput(bool value);
  bool sendStandardAnalogOuput(uint8_t output_pin, bool value);

private:
  comm::URStream<PackageHeader>* stream_;
  std::vector<std::string> recipe_;
  uint8_t recipe_id_;
};

}  // namespace rtde_interface
}  // namespace ur_driver

#endif  // UR_RTDE_DRIVER_RTDE_WRITER_H_INCLUDED
