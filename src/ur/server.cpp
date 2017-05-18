#include <cstring>
#include <netinet/tcp.h>
#include <unistd.h>
#include "ur_modern_driver/log.h"
#include "ur_modern_driver/ur/server.h"

URServer::URServer(int port)
  : port_(port)
{
}

void URServer::setOptions(int socket_fd)
{
  TCPSocket::setOptions(socket_fd);

  int flag = 1;
  setsockopt(socket_fd, SOL_SOCKET, SO_REUSEADDR, &flag, sizeof(int));
}

std::string URServer::getIP()
{
  char buf[128];
  int res = ::gethostname(buf, sizeof(buf));
  return std::string(buf);
}

bool URServer::bind()
{
  std::string empty;
  bool res = TCPSocket::setup(empty, port_);
  state_ = TCPSocket::getState();
  
  if(!res)
    return false;

  if(::listen(getSocketFD(), 1) < 0)
    return false;

  return true;
}

bool URServer::accept()
{
  if(state_ != SocketState::Connected || client_.getSocketFD() > 0)
    return false;

  struct sockaddr addr; 
  socklen_t addr_len;
  int client_fd = ::accept(getSocketFD(), &addr, &addr_len);

  if(client_fd <= 0)
    return false;

  return client_.setSocketFD(client_fd);
}

bool URServer::write(const uint8_t* buf, size_t buf_len, size_t &written)
{
  return client_.write(buf, buf_len, written);
}