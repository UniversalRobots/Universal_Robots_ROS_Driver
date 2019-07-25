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

#include "ur_rtde_driver/rtde/rtde_writer.h"

namespace ur_driver
{
namespace rtde_interface
{
RTDEWriter::RTDEWriter(comm::URStream<PackageHeader>* stream, const std::string& recipe_file) : stream_(stream)
{
}

bool RTDEWriter::init(uint8_t recipe_id)
{
  return false;
}
bool RTDEWriter::start()
{
  return false;
}

bool RTDEWriter::sendSpeedSlider(double speed_slider_fraction)
{
  return false;
}
bool RTDEWriter::sendStandardDigitalOutput(uint8_t output_pin, bool value)
{
  return false;
}
bool RTDEWriter::sendConfigurableDigitalOutput(uint8_t output_pin, bool value)
{
  return false;
}
bool RTDEWriter::sendToolDigitalOutput(bool value)
{
  return false;
}
bool RTDEWriter::sendStandardAnalogOuput(uint8_t output_pin, bool value)
{
  return false;
}

}  // namespace rtde_interface
}  // namespace ur_driver
