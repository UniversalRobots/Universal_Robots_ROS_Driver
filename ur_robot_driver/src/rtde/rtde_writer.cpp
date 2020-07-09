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

#include "ur_robot_driver/rtde/rtde_writer.h"

namespace ur_driver
{
namespace rtde_interface
{
RTDEWriter::RTDEWriter(comm::URStream<RTDEPackage>* stream, const std::vector<std::string>& recipe)
  : stream_(stream), recipe_(recipe), queue_{ 32 }, running_(false)
{
}

void RTDEWriter::init(uint8_t recipe_id)
{
  recipe_id_ = recipe_id;
  running_ = true;
  writer_thread_ = std::thread(&RTDEWriter::run, this);
}

void RTDEWriter::run()
{
  uint8_t buffer[4096];
  size_t size;
  size_t written;
  std::unique_ptr<DataPackage> package;
  while (running_)
  {
    if (queue_.waitDequeTimed(package, 1000000))
    {
      package->setRecipeID(recipe_id_);
      size = package->serializePackage(buffer);
      stream_->write(buffer, size, written);
    }
  }
  LOG_DEBUG("Write thread ended.");
}

bool RTDEWriter::sendSpeedSlider(double speed_slider_fraction)
{
  std::unique_ptr<DataPackage> package;
  package.reset(new DataPackage(recipe_));
  package->initEmpty();
  uint32_t mask = 1;
  bool success = true;
  success = package->setData("speed_slider_mask", mask);
  success = success && package->setData("speed_slider_fraction", speed_slider_fraction);

  if (success)
  {
    if (!queue_.tryEnqueue(std::move(package)))
    {
      return false;
    }
  }
  return success;
}

bool RTDEWriter::sendStandardDigitalOutput(uint8_t output_pin, bool value)
{
  std::unique_ptr<DataPackage> package;
  package.reset(new DataPackage(recipe_));
  package->initEmpty();
  uint8_t mask = pinToMask(output_pin);
  bool success = true;
  uint8_t digital_output;
  if (value)
  {
    digital_output = 255;
  }
  else
  {
    digital_output = 0;
  }
  success = package->setData("standard_digital_output_mask", mask);
  success = success && package->setData("standard_digital_output", digital_output);

  if (success)
  {
    if (!queue_.tryEnqueue(std::move(package)))
    {
      return false;
    }
  }
  return success;
}

bool RTDEWriter::sendConfigurableDigitalOutput(uint8_t output_pin, bool value)
{
  std::unique_ptr<DataPackage> package;
  package.reset(new DataPackage(recipe_));
  package->initEmpty();
  uint8_t mask = pinToMask(output_pin);
  bool success = true;
  uint8_t digital_output;
  if (value)
  {
    digital_output = 255;
  }
  else
  {
    digital_output = 0;
  }
  success = package->setData("configurable_digital_output_mask", mask);
  success = success && package->setData("configurable_digital_output", digital_output);

  if (success)
  {
    if (!queue_.tryEnqueue(std::move(package)))
    {
      return false;
    }
  }
  return success;
}

bool RTDEWriter::sendToolDigitalOutput(uint8_t output_pin, bool value)
{
  std::unique_ptr<DataPackage> package;
  package.reset(new DataPackage(recipe_));
  package->initEmpty();
  uint8_t mask = pinToMask(output_pin);
  bool success = true;
  uint8_t digital_output;
  if (value)
  {
    digital_output = 255;
  }
  else
  {
    digital_output = 0;
  }
  success = package->setData("tool_digital_output_mask", mask);
  success = success && package->setData("tool_digital_output", digital_output);

  if (success)
  {
    if (!queue_.tryEnqueue(std::move(package)))
    {
      return false;
    }
  }
  return success;
}

bool RTDEWriter::sendStandardAnalogOutput(uint8_t output_pin, double value)
{
  std::unique_ptr<DataPackage> package;
  package.reset(new DataPackage(recipe_));
  package->initEmpty();
  uint8_t mask = pinToMask(output_pin);
  // default to current for now, as no functionality to choose included in set io service
  uint8_t output_type = 0;
  bool success = true;
  success = package->setData("standard_analog_output_mask", mask);
  success = success && package->setData("standard_analog_output_type", output_type);
  success = success && package->setData("standard_analog_output_0", value);
  success = success && package->setData("standard_analog_output_1", value);

  if (success)
  {
    if (!queue_.tryEnqueue(std::move(package)))
    {
      return false;
    }
  }
  return success;
}

uint8_t RTDEWriter::pinToMask(uint8_t pin)
{
  if (pin > 7)
  {
    return 0;
  }

  return 1 << pin;
}

}  // namespace rtde_interface
}  // namespace ur_driver
