#pragma once
#include <netdb.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <atomic>
#include <string>

/// Encapsulates a TCP socket
class URStream
{
private:
  int socket_fd_ = -1;
  std::string host_;
  int port_;

  std::atomic<bool> initialized_;
  std::atomic<bool> stopping_;

public:
  URStream(std::string& host, int port) : host_(host), port_(port), initialized_(false), stopping_(false)
  {
  }

  ~URStream()
  {
    disconnect();
  }

  bool connect();
  void disconnect();

  ssize_t send(uint8_t* buf, size_t buf_len);
  ssize_t receive(uint8_t* buf, size_t buf_len);
};