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
#include <mutex>
#include <string>
#include <memory>

namespace ur_driver
{
namespace comm
{
/*!
 * \brief State the socket can be in
 */
enum class SocketState
{
  Invalid,       ///< Socket is initialized or setup failed
  Connected,     ///< Socket is connected and ready to use
  Disconnected,  ///< Socket is disconnected and cannot be used
  Closed         ///< Connection to socket got closed
};

/*!
 * \brief Class for TCP socket abstraction
 */
class TCPSocket
{
private:
  std::atomic<int> socket_fd_;
  std::atomic<SocketState> state_;

protected:
  virtual bool open(int socket_fd, struct sockaddr* address, size_t address_len)
  {
    return false;
  }
  virtual void setOptions(int socket_fd);

  bool setup(std::string& host, int port);

  std::unique_ptr<timeval> recv_timeout_;

public:
  /*!
   * \brief Creates a TCPSocket object
   */
  TCPSocket();
  virtual ~TCPSocket();

  /*!
   * \brief Getter for the state of the socket.
   *
   * \returns Returns the current state of the socket
   */
  SocketState getState()
  {
    return state_;
  }

  /*!
   * \brief Getter for the file descriptor of the socket.
   *
   * \returns The file descriptor of the socket
   */
  int getSocketFD()
  {
    return socket_fd_;
  }
  /*!
   * \brief Setter for the file descriptor of the socket.
   *
   * \param socket_fd The new value
   *
   * \returns False, if the socket is in state connected
   */
  bool setSocketFD(int socket_fd);

  /*!
   * \brief Determines the IP address of the local machine
   *
   * \returns The IP address of the local machine.
   */
  std::string getIP() const;

  /*!
   * \brief Reads one byte from the socket
   *
   * \param[out] character Target buffer
   *
   * \returns True on success, false otherwise
   */
  bool read(char* character);

  /*!
   * \brief Reads data from the socket
   *
   * \param[out] buf Buffer where the data shall be stored
   * \param[in] buf_len Number of bytes allocated for the buffer
   * \param[out] read Number of bytes actually read
   *
   * \returns True on success, false otherwise
   */
  bool read(uint8_t* buf, const size_t buf_len, size_t& read);

  /*!
   * \brief Writes to the socket
   *
   * \param[in] buf Buffer of bytes to write
   * \param[in] buf_len Number of bytes in the buffer
   * \param[out] written Number of bytes actually written
   *
   * \returns True on success, false otherwise
   */
  bool write(const uint8_t* buf, const size_t buf_len, size_t& written);

  /*!
   * \brief Closes the connection to the socket.
   */
  void close();

  /*!
   * \brief Setup Receive timeout used for this socket.
   *
   * \param timeout Timeout used for setting things up
   */
  void setReceiveTimeout(const timeval& timeout);
};
}  // namespace comm
}  // namespace ur_driver
