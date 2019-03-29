/*
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
#include "ur_modern_driver/log.h"
#include "ur_modern_driver/tcp_socket.h"

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