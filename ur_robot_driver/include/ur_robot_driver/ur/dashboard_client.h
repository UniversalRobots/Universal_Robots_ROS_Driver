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
 * \author  Felix Mauch mauch@fzi.de
 * \date    2019-10-21
 *
 */
//----------------------------------------------------------------------
#ifndef UR_ROBOT_DRIVER_DASHBOARD_CLIENT_DASHBOARD_CLIENT_H_INCLUDED
#define UR_ROBOT_DRIVER_DASHBOARD_CLIENT_DASHBOARD_CLIENT_H_INCLUDED

#include <ur_robot_driver/comm/tcp_socket.h>

namespace ur_driver
{
class DashboardClient : public comm::TCPSocket
{
public:
  DashboardClient(const std::string& host, int port);
  virtual ~DashboardClient() = default;

  enum class OperationalMode {
    MANUAL,
    AUTOMATIC
  };

  bool connect();
  void disconnect();

  // Sending functions
  bool addToLog(const std::string& text);
  bool brakeRelease();
  bool clearOperationalMode();
  bool closePopup();
  bool closeSafetyPopup();
  bool loadInstallation(const std::string& installation_name);
  bool loadProgram(const std::string& program_name);
  bool pause();
  bool play();
  bool popup(const std::string& text);
  bool powerOff();
  bool powerOn();
  bool quit();
  bool restartSafety();
  bool setOperationalMode(const OperationalMode mode);
  bool shutdown();
  bool stop();
  bool unlockProtectiveStop();

  // Requesting information
  bool running();
  std::string robotMode();
  std::string getLoadedProgram();
  bool isProgramSaved();
  std::string programState();
  std::string polyscopeVersion();
  std::string safetyMode();
  std::string safetyStatus();

protected:
  virtual bool open(int socket_fd, struct sockaddr* address, size_t address_len)
  {
    return ::connect(socket_fd, address, address_len) == 0;
  }

private:
  bool send(const std::string& text);
  std::string read();
  std::string sendAndReceive(const std::string& text);

  std::string host_;
  int port_;
  std::mutex write_mutex_, read_mutex_;
};
}  // namespace ur_driver
#endif  // ifndef UR_ROBOT_DRIVER_DASHBOARD_CLIENT_DASHBOARD_CLIENT_H_INCLUDED
