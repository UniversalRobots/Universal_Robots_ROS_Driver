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
 * \author  Felix Exner exner@fzi.de
 * \date    2019-10-21
 *
 */
//----------------------------------------------------------------------
#ifndef UR_ROBOT_DRIVER_DASHBOARD_CLIENT_DASHBOARD_CLIENT_H_INCLUDED
#define UR_ROBOT_DRIVER_DASHBOARD_CLIENT_DASHBOARD_CLIENT_H_INCLUDED

#include <ur_robot_driver/comm/tcp_socket.h>

namespace ur_driver
{
/*!
 * \brief This class is a wrapper around the dashboard server.
 *
 * For every Dashboard command there exists a wrapper function that will send the request and wait
 * for the server's response.
 *
 * For documentation about the dashboard server, please see
 *  - https://www.universal-robots.com/how-tos-and-faqs/how-to/ur-how-tos/dashboard-server-cb-series-port-29999-15690/
 *  - https://www.universal-robots.com/how-tos-and-faqs/how-to/ur-how-tos/dashboard-server-e-series-port-29999-42728/
 */
class DashboardClient : public comm::TCPSocket
{
public:
  /*!
   * \brief Constructor that shall be used by default
   *
   * \param host IP address of the robot
   */
  DashboardClient(const std::string& host);
  DashboardClient() = delete;
  virtual ~DashboardClient() = default;

  const int DASHBOARD_SERVER_PORT = 29999;

  /*!
   * \brief Opens a connection to the dasboard server on the host as specified in the constructor.
   *
   * \returns True on successful connection, false otherwise.
   */
  bool connect();

  /*!
   * \brief Makes sure no connection to the dashboard server is held inside the object.
   */
  void disconnect();

  /*!
   * \brief Sends a command through the socket and waits for an answer.
   *
   * \param command Command that will be sent to the server. It is important, that the
   * command sent is finished with a '\n' (newline) so it will be processed by the server.
   *
   * \returns Answer as received by the server cut off any trailing newlines.
   */
  std::string sendAndReceive(const std::string& command);

protected:
  virtual bool open(int socket_fd, struct sockaddr* address, size_t address_len)
  {
    return ::connect(socket_fd, address, address_len) == 0;
  }

private:
  bool send(const std::string& text);
  std::string read();

  void rtrim(std::string& str, const std::string& chars = "\t\n\v\f\r ")
  {
    str.erase(str.find_last_not_of(chars) + 1);
  }

  std::string host_;
  int port_;
  std::mutex write_mutex_;
};
}  // namespace ur_driver
#endif  // ifndef UR_ROBOT_DRIVER_DASHBOARD_CLIENT_DASHBOARD_CLIENT_H_INCLUDED
