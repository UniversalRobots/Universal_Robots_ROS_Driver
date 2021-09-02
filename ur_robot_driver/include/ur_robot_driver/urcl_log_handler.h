// -- BEGIN LICENSE BLOCK ----------------------------------------------
// Copyright 2021 Universal Robots A/S
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

// All source code contained in and/or linked to in this message (the “Source Code”) is subject to the copyright of
// Universal Robots A/S and/or its licensors. THE SOURCE CODE IS PROVIDED “AS IS” WITHOUT WARRANTY OF ANY KIND, EXPRESS
// OR IMPLIED, INCLUDING – BUT NOT LIMITED TO – WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE, OR
// NONINFRINGEMENT. USE OF THE SOURCE CODE IS AT YOUR OWN RISK AND UNIVERSAL ROBOTS A/S AND ITS LICENSORS SHALL, TO THE
// MAXIMUM EXTENT PERMITTED BY LAW, NOT BE LIABLE FOR ANY ERRORS OR MALICIOUS CODE IN THE SOURCE CODE, ANY THIRD-PARTY
// CLAIMS, OR ANY OTHER CLAIMS AND DAMAGES, INCLUDING INDIRECT, INCIDENTAL, SPECIAL, CONSEQUENTIAL OR PUNITIVE DAMAGES,
// OR ANY LOSS OF PROFITS, EXPECTED SAVINGS, OR REVENUES, WHETHER INCURRED DIRECTLY OR INDIRECTLY, OR ANY LOSS OF DATA,
// USE, GOODWILL, OR OTHER INTANGIBLE LOSSES, RESULTING FROM YOUR USE OF THE SOURCE CODE. You may make copies of the
// Source Code for use in connection with a Universal Robots or UR+ product, provided that you include (i) an
// appropriate copyright notice (“©  [the year in which you received the Source Code or the Source Code was first
// published, e.g. “2021”] Universal Robots A/S and/or its licensors”) along with the capitalized section of this notice
// in all copies of the Source Code. By using the Source Code, you agree to the above terms. For more information,
// please contact legal@universal-robots.com.
// -- END LICENSE BLOCK ------------------------------------------------

#ifndef UR_DRIVER_URCL_LOG_HANDLER_H_INCLUDED
#define UR_DRIVER_URCL_LOG_HANDLER_H_INCLUDED

#include <ur_client_library/log.h>

namespace ur_driver
{
/*!
 * \brief Loghandler for handling messages logged with the C++ client library. This loghandler will log the messages
 * from the client library with ROS logging.
 * Use registerLogHandler to register this class. This class shouldn't be instantiated directly.
 */
class UrclLogHandler : public urcl::LogHandler
{
public:
  /*!
   * \brief Default constructor
   */
  UrclLogHandler();

  /*!
   * \brief Function to log a message
   *
   * \param file The log message comes from this file
   * \param line The log message comes from this line
   * \param loglevel Indicates the severity of the log message
   * \param log Log message
   */
  void log(const char* file, int line, urcl::LogLevel loglevel, const char* message) override;

private:
  std::string log_name_;

  void logMessage(const char* file, int line, ros::console::Level level, const char* message);
};

/*!
 * \brief Register the UrclLoghHandler, this will start logging messages from the client library with RPS2 logging.
 * This function has to be called inside your node, to enable the log handler.
 */
void registerUrclLogHandler();

/*!
 * \brief Unregister the UrclLoghHandler, stop logging messages from the client library with ROS2 logging.
 */
void unregisterUrclLogHandler();

}  // namespace ur_driver

#endif  // UR_DRIVER_URCL_LOG_HANDLER_H_INCLUDED
