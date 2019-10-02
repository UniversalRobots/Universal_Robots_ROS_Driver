/*
 * Copyright 2019, FZI Forschungszentrum Informatik (refactor)
 *
 * Copyright 2017, 2018 Simon Rasmussen (refactor)
 *
 * Copyright 2015, 2016 Thomas Timm Andersen (original version)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#pragma once
#include <netdb.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <atomic>
#include <cstdlib>
#include <mutex>
#include <string>
#include "ur_robot_driver/comm/tcp_socket.h"

namespace ur_driver
{
namespace comm
{
#define MAX_SERVER_BUF_LEN 50

/*!
 * \brief The URServer class abstracts communication with the robot. It opens a socket on a given
 * port and waits for a robot to connect, at which point two way communication can be established.
 */
class URServer : private comm::TCPSocket
{
private:
  int port_;
  comm::TCPSocket client_;

protected:
  virtual bool open(int socket_fd, struct sockaddr* address, size_t address_len);

public:
  /*!
   * \brief Creates a URServer object with a given port.
   *
   * \param port The port to open a socket on
   */
  URServer(int port);
  /*!
   * \brief Closes the socket to allow for destruction of the object.
   */
  ~URServer();
  /*!
   * \brief Getter for the server IP.
   *
   * \returns The IP of the server
   */
  std::string getIP();
  /*!
   * \brief Binds to server's port, setting up a socket if possible.
   *
   * \returns Success of setting up the socket
   */
  bool bind();
  /*!
   * \brief Waits for a robot to connect to the socket.
   *
   * \returns True, if a robot successfully connected, false otherwise.
   */
  bool accept();
  /*!
   * \brief Triggers a disconnect of the currently connected robot.
   */
  void disconnectClient();
  /*!
   * \brief Reads the byte-stream from the robot to the next linebreak.
   *
   * \param buffer The buffer to write the received bytes to
   * \param buf_len Size of the buffer
   *
   * \returns True if a successful read occurred, false otherwise
   */
  bool readLine(char* buffer, size_t buf_len);
  /*!
   * \brief Writes a buffer to the robot.
   *
   * \param buf The buffer to write from
   * \param buf_len The length to write
   * \param written A reference used to indicate how many bytes were written
   *
   * \returns Success of the write
   */
  bool write(const uint8_t* buf, size_t buf_len, size_t& written);
};
}  // namespace comm
}  // namespace ur_driver
