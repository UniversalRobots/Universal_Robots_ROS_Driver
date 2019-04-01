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

#include <endian.h>
#include <netinet/tcp.h>
#include <unistd.h>
#include <cstring>

#include "ur_modern_driver/log.h"
#include "ur_modern_driver/ur/stream.h"

bool URStream::write(const uint8_t* buf, size_t buf_len, size_t& written)
{
  std::lock_guard<std::mutex> lock(write_mutex_);
  return TCPSocket::write(buf, buf_len, written);
}

bool URStream::read(uint8_t* buf, size_t buf_len, size_t& total)
{
  std::lock_guard<std::mutex> lock(read_mutex_);

  bool initial = true;
  uint8_t* buf_pos = buf;
  size_t remainder = sizeof(int32_t);
  size_t read = 0;

  while (remainder > 0 && TCPSocket::read(buf_pos, remainder, read))
  {
    TCPSocket::setOptions(getSocketFD());
    if (initial)
    {
      remainder = be32toh(*(reinterpret_cast<int32_t*>(buf)));
      if (remainder >= (buf_len - sizeof(int32_t)))
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