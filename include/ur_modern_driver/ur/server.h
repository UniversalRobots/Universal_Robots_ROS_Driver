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
