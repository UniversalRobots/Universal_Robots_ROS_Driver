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