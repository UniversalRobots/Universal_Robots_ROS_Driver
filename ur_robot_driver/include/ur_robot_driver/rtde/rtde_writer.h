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

#include "ur_robot_driver/rtde/package_header.h"
#include "ur_robot_driver/rtde/rtde_package.h"
#include "ur_robot_driver/rtde/data_package.h"
#include "ur_robot_driver/comm/stream.h"
#include "ur_robot_driver/queue/readerwriterqueue.h"
#include <thread>

namespace ur_driver
{
namespace rtde_interface
{
/*!
 * \brief The RTDEWriter class offers an abstraction layer to send data to the robot via the RTDE
 * interface. Several simple to use functions to create data packages to send exist, which are
 * then sent to the robot in an additional thread.
 */
class RTDEWriter
{
public:
  RTDEWriter() = delete;
  /*!
   * \brief Creates a new RTDEWriter object using a given URStream and recipe.
   *
   * \param stream The URStream to use for communication with the robot
   * \param recipe The recipe to use for communication
   */
  RTDEWriter(comm::URStream<RTDEPackage>* stream, const std::vector<std::string>& recipe);

  ~RTDEWriter()
  {
    running_ = false;
    std::this_thread::sleep_for(std::chrono::seconds(5));
    if (writer_thread_.joinable())
    {
      writer_thread_.join();
    }
  }
  /*!
   * \brief Starts the writer thread, which periodically clears the queue to write packages to the
   * robot.
   *
   * \param recipe_id The recipe id to use, so the robot correctly identifies the used recipe
   */
  void init(uint8_t recipe_id);
  /*!
   * \brief The writer thread loop, continually serializing and sending packages to the robot.
   */
  void run();

  /*!
   * \brief Creates a package to request setting a new value for the speed slider.
   *
   * \param speed_slider_fraction The new speed slider fraction as a value between 0.0 and 1.0
   *
   * \returns Success of the package creation
   */
  bool sendSpeedSlider(double speed_slider_fraction);
  /*!
   * \brief Creates a package to request setting a new value for one of the standard digital output pins.
   *
   * \param output_pin The pin to change
   * \param value The new value
   *
   * \returns Success of the package creation
   */
  bool sendStandardDigitalOutput(uint8_t output_pin, bool value);
  /*!
   * \brief Creates a package to request setting a new value for one of the configurable digital output pins.
   *
   * \param output_pin The pin to change
   * \param value The new value
   *
   * \returns Success of the package creation
   */
  bool sendConfigurableDigitalOutput(uint8_t output_pin, bool value);
  /*!
   * \brief Creates a package to request setting a new value for one of the tool output pins.
   *
   * \param output_pin The pin to change
   * \param value The new value
   *
   * \returns Success of the package creation
   */
  bool sendToolDigitalOutput(uint8_t output_pin, bool value);
  /*!
   * \brief Creates a package to request setting a new value for one of the standard analog output pins.
   *
   * \param output_pin The pin to change
   * \param value The new value
   *
   * \returns Success of the package creation
   */
  bool sendStandardAnalogOutput(uint8_t output_pin, double value);

private:
  uint8_t pinToMask(uint8_t pin);
  comm::URStream<RTDEPackage>* stream_;
  std::vector<std::string> recipe_;
  uint8_t recipe_id_;
  moodycamel::BlockingReaderWriterQueue<std::unique_ptr<DataPackage>> queue_;
  std::thread writer_thread_;
  bool running_;
};

}  // namespace rtde_interface
}  // namespace ur_driver

#endif  // UR_RTDE_DRIVER_RTDE_WRITER_H_INCLUDED
