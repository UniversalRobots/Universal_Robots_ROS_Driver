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
#include "ur_rtde_driver/log.h"
#include "ur_rtde_driver/comm/tcp_socket.h"

namespace ur_driver
{
namespace comm
{
template <typename HeaderT>
class URStream : public TCPSocket
{
private:
  std::string host_;
  int port_;
  std::mutex write_mutex_, read_mutex_;

protected:
  virtual bool open(int socket_fd, struct sockaddr* address, size_t address_len)
  {
    return ::connect(socket_fd, address, address_len) == 0;
  }

public:
  URStream(std::string& host, int port) : host_(host), port_(port)
  {
  }

  bool connect()
  {
    return TCPSocket::setup(host_, port_);
  }
  void disconnect()
  {
    LOG_INFO("Disconnecting from %s:%d", host_.c_str(), port_);
    TCPSocket::close();
  }

  bool closed()
  {
    return getState() == SocketState::Closed;
  }

  bool read(uint8_t* buf, size_t buf_len, size_t& read);
  bool write(const uint8_t* buf, size_t buf_len, size_t& written);
};

template <typename HeaderT>
bool URStream<HeaderT>::write(const uint8_t* buf, size_t buf_len, size_t& written)
{
  std::lock_guard<std::mutex> lock(write_mutex_);
  return TCPSocket::write(buf, buf_len, written);
}

template <typename HeaderT>
bool URStream<HeaderT>::read(uint8_t* buf, size_t buf_len, size_t& total)
{
  std::lock_guard<std::mutex> lock(read_mutex_);

  bool initial = true;
  uint8_t* buf_pos = buf;
  size_t remainder = sizeof(HeaderT::_package_size_type);
  size_t read = 0;

  while (remainder > 0 && TCPSocket::read(buf_pos, remainder, read))
  {
    TCPSocket::setOptions(getSocketFD());
    if (initial)
    {
      remainder = HeaderT::getPackageLength(buf);
      if (remainder >= (buf_len - sizeof(HeaderT::_package_size_type)))
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
