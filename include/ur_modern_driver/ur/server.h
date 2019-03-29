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
#include <cstdlib>
#include <mutex>
#include <string>
#include "ur_modern_driver/tcp_socket.h"

#define MAX_SERVER_BUF_LEN 50

class URServer : private TCPSocket
{
private:
  int port_;
  TCPSocket client_;

protected:
  virtual bool open(int socket_fd, struct sockaddr *address, size_t address_len);

public:
  URServer(int port);
  ~URServer();
  std::string getIP();
  bool bind();
  bool accept();
  void disconnectClient();
  bool readLine(char *buffer, size_t buf_len);
  bool write(const uint8_t *buf, size_t buf_len, size_t &written);
};
