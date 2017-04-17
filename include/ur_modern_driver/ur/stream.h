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
  URStream() 
  {
  }

  URStream(std::string& host, int port) : host_(host), port_(port), initialized_(false), stopping_(false)
  {
  }

  URStream(int socket_fd) : socket_fd_(socket_fd), initialized_(true), stopping_(false)
  {
    
  }

  URStream(URStream&& other) noexcept : socket_fd_(other.socket_fd_), host_(other.host_), initialized_(other.initialized_.load()), stopping_(other.stopping_.load())
  {
    
  }

  ~URStream()
  {
    disconnect();
  }

  URStream& operator=(URStream&& other)
  {
    socket_fd_ = std::move(other.socket_fd_);
    host_ = std::move(other.host_); 
    initialized_ = std::move(other.initialized_.load());
    stopping_ = std::move(other.stopping_.load());
    return *this;
  }

  bool connect();
  void disconnect();
  void reconnect();

  ssize_t send(const uint8_t* buf, size_t buf_len);
  ssize_t receive(uint8_t* buf, size_t buf_len);
};