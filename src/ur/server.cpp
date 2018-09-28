#include "ur_modern_driver/ur/server.h"
#include <arpa/inet.h>
#include <netinet/tcp.h>
#include <unistd.h>
#include <cstring>
#include "ur_modern_driver/log.h"

URServer::URServer(int port) : port_(port)
{
}

URServer::~URServer()
{
  TCPSocket::close();
}

std::string URServer::getIP()
{
  sockaddr_in name;
  socklen_t len = sizeof(name);
  int res = ::getsockname(getSocketFD(), (sockaddr*)&name, &len);

  if (res < 0)
  {
    LOG_ERROR("Could not get local IP");
    return std::string();
  }

  char buf[128];
  inet_ntop(AF_INET, &name.sin_addr, buf, sizeof(buf));
  return std::string(buf);
}


bool URServer::open(int socket_fd, struct sockaddr *address, size_t address_len)
{
  int flag = 1;
  setsockopt(socket_fd, SOL_SOCKET, SO_REUSEADDR, &flag, sizeof(int));
  return ::bind(socket_fd, address, address_len) == 0;
}

bool URServer::bind()
{
  std::string empty;
  bool res = TCPSocket::setup(empty, port_);

  if (!res)
    return false;

  if (::listen(getSocketFD(), 1) < 0)
    return false;

  return true;
}

bool URServer::accept()
{
  if (TCPSocket::getState() != SocketState::Connected || client_.getSocketFD() > 0)
    return false;

  struct sockaddr addr;
  socklen_t addr_len;
  int client_fd = -1;

  int retry = 0;
  while((client_fd = ::accept(getSocketFD(), &addr, &addr_len)) == -1){
    LOG_ERROR("Accepting socket connection failed. (errno: %d)", errno);
    if(retry++ >= 5)
      return false;
  }

  TCPSocket::setOptions(client_fd);

  return client_.setSocketFD(client_fd);
}

void URServer::disconnectClient()
{
  if (client_.getState() != SocketState::Connected)
    return;

  client_.close();
}

bool URServer::write(const uint8_t* buf, size_t buf_len, size_t& written)
{
  return client_.write(buf, buf_len, written);
}

bool URServer::readLine(char* buffer, size_t buf_len)
{
    char *current_pointer = buffer;
    char ch;
    size_t total_read;

    if (buf_len <= 0 || buffer == NULL) {
        return false;
    }

    total_read = 0;
    for (;;) {
        if (client_.read(&ch))
        {
            if (total_read < buf_len - 1) // just in case ...
            {
                total_read ++;
                *current_pointer++ = ch;
            }
            if (ch == '\n')
            {
                break;
            }
        }
        else
        {
            if (total_read == 0)
            {
                return false;
            }
            else
            {
                break;
            }
        }
    }

    *current_pointer = '\0';
    return true;
}
