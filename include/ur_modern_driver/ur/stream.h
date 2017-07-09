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