/*
 * Copyright 2019, FZI Forschungszentrum Informatik (templating)
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
#include "ur_robot_driver/log.h"
#include "ur_robot_driver/comm/tcp_socket.h"

namespace ur_driver
{
namespace comm
{
/*!
 * \brief The stream is an abstraction of the TCPSocket that offers reading a full UR data package
 * out of the socket. This means, it has to have some knowledge about the package structure to
 * peek at the field defining the package length. This is why it is templated with the package
 * header type.
 */
template <typename T>
class URStream : public TCPSocket
{
public:
  /*!
   * \brief Creates a new URStream object. Note, that this does not immediately open the socket,
   * that has to be done separately by calling the connect() function.
   *
   * \param host IP address of the remote host
   * \param port Port on which the socket shall be connected
   */
  URStream(const std::string& host, int port) : host_(host), port_(port)
  {
  }

  /*!
   * \brief Connects to the configured socket.
   *
   * \returns True on success, false if connection could not be established
   */
  bool connect()
  {
    return TCPSocket::setup(host_, port_);
  }

  /*!
   * \brief Disconnects from the configured socket.
   */
  void disconnect()
  {
    LOG_DEBUG("Disconnecting from %s:%d", host_.c_str(), port_);
    TCPSocket::close();
  }

  /*!
   * \brief Returns whether the underlying socket is currently closed
   */
  bool closed()
  {
    return getState() == SocketState::Closed;
  }

  /*!
   * \brief Reads a full UR package out of a socket. For this, it looks into the package and reads
   * the byte length from the socket directly. It returns as soon as all bytes for the package are
   * read from the socket.
   *
   * \param[out] buf The byte buffer where the content shall be stored
   * \param[in] buf_len Number of bytes allocated for the buffer
   * \param[out] read Number of bytes actually read from the socket
   *
   * \returns True on success, false on error, e.g. the buffer is smaller than the package size
   */
  bool read(uint8_t* buf, const size_t buf_len, size_t& read);

  /*!
   * \brief Writes directly to the underlying socket (with a mutex guard)
   *
   * \param[in] buf Byte stream that should be sent
   * \param[in] buf_len Number of bytes in buffer
   * \param[out] written Number of bytes actually written to the socket
   *
   * \returns False if sending went wrong
   */
  bool write(const uint8_t* buf, const size_t buf_len, size_t& written);

protected:
  virtual bool open(int socket_fd, struct sockaddr* address, size_t address_len)
  {
    return ::connect(socket_fd, address, address_len) == 0;
  }

private:
  std::string host_;
  int port_;
  std::mutex write_mutex_, read_mutex_;
};

template <typename T>
bool URStream<T>::write(const uint8_t* buf, const size_t buf_len, size_t& written)
{
  std::lock_guard<std::mutex> lock(write_mutex_);
  return TCPSocket::write(buf, buf_len, written);
}

template <typename T>
bool URStream<T>::read(uint8_t* buf, const size_t buf_len, size_t& total)
{
  std::lock_guard<std::mutex> lock(read_mutex_);

  bool initial = true;
  uint8_t* buf_pos = buf;
  size_t remainder = sizeof(typename T::HeaderType::_package_size_type);
  size_t read = 0;

  while (remainder > 0 && TCPSocket::read(buf_pos, remainder, read))
  {
    TCPSocket::setOptions(getSocketFD());
    if (initial)
    {
      remainder = T::HeaderType::getPackageLength(buf);
      if (remainder >= (buf_len - sizeof(typename T::HeaderType::_package_size_type)))
      {
        LOG_ERROR("Packet size %zd is larger than buffer %zu, discarding.", remainder, buf_len);
        return false;
      }
      initial = false;
    }

    total += read;
    buf_pos += read;
    remainder -= read;
  }

  return remainder == 0;
}
}  // namespace comm
}  // namespace ur_driver
