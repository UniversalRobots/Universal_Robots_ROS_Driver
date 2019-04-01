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

enum class SocketState
{
  Invalid,
  Connected,
  Disconnected,
  Closed
};

class TCPSocket
{
private:
  std::atomic<int> socket_fd_;
  std::atomic<SocketState> state_;

protected:
  virtual bool open(int socket_fd, struct sockaddr *address, size_t address_len)
  {
    return false;
  }
  virtual void setOptions(int socket_fd);

  bool setup(std::string &host, int port);

public:
  TCPSocket();
  virtual ~TCPSocket();

  SocketState getState()
  {
    return state_;
  }

  int getSocketFD()
  {
    return socket_fd_;
  }
  bool setSocketFD(int socket_fd);

  std::string getIP();

  bool read(char *character);
  bool read(uint8_t *buf, size_t buf_len, size_t &read);
  bool write(const uint8_t *buf, size_t buf_len, size_t &written);

  void close();
};
