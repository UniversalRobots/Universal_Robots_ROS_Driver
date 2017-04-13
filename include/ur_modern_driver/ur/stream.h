#pragma once
#include <netdb.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <mutex>
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
  std::mutex send_mutex_, receive_mutex_;

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
  void reconnect();

  ssize_t send(const uint8_t* buf, size_t buf_len);
  ssize_t receive(uint8_t* buf, size_t buf_len);
};