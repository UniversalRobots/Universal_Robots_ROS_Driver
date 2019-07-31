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
RTDEWriter::RTDEWriter(comm::URStream<PackageHeader>* stream, const std::vector<std::string>& recipe)
  : stream_(stream), recipe_(recipe), queue_{ 32 }
{
}

void RTDEWriter::init(uint8_t recipe_id)
{
  recipe_id_ = recipe_id;
  writer_thread_ = std::thread(&RTDEWriter::run, this);
}

void RTDEWriter::run()
{
  uint8_t buffer[4096];
  size_t size;
  size_t written;
  std::unique_ptr<DataPackage> package;
  while (true)
  {
    queue_.waitDequeue(package);
    package->setRecipeID(recipe_id_);
    size = package->serializePackage(buffer);
    stream_->write(buffer, size, written);
  }
}

bool RTDEWriter::sendSpeedSlider(double speed_slider_fraction)
{
  std::unique_ptr<DataPackage> package;
  package.reset(new DataPackage(recipe_));
  package->initEmpty();
  uint32_t mask = 1;
  package->setData("speed_slider_mask", mask);
  package->setData("speed_slider_fraction", speed_slider_fraction);

  if (!queue_.tryEnqueue(std::move(package)))
  {
    return false;
  }
  return true;
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
