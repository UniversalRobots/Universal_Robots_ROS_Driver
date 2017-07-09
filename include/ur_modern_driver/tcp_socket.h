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

  bool read(uint8_t *buf, size_t buf_len, size_t &read);
  bool write(const uint8_t *buf, size_t buf_len, size_t &written);

  void close();
};
