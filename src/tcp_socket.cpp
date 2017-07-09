#include <arpa/inet.h>
#include <endian.h>
#include <netinet/tcp.h>
#include <unistd.h>
#include <cstring>

#include "ur_modern_driver/log.h"
#include "ur_modern_driver/tcp_socket.h"

TCPSocket::TCPSocket() : socket_fd_(-1), state_(SocketState::Invalid)
{
}
TCPSocket::~TCPSocket()
{
  close();
}

void TCPSocket::setOptions(int socket_fd)
{
  int flag = 1;
  setsockopt(socket_fd, IPPROTO_TCP, TCP_NODELAY, &flag, sizeof(int));
  setsockopt(socket_fd, IPPROTO_TCP, TCP_QUICKACK, &flag, sizeof(int));
}

bool TCPSocket::setup(std::string &host, int port)
{
  if (state_ == SocketState::Connected)
    return false;

  LOG_INFO("Setting up connection: %s:%d", host.c_str(), port);

  // gethostbyname() is deprecated so use getadderinfo() as described in:
  // http://www.beej.us/guide/bgnet/output/html/multipage/syscalls.html#getaddrinfo

  const char *host_name = host.empty() ? nullptr : host.c_str();
  std::string service = std::to_string(port);
  struct addrinfo hints, *result;
  std::memset(&hints, 0, sizeof(hints));

  hints.ai_family = AF_UNSPEC;
  hints.ai_socktype = SOCK_STREAM;
  hints.ai_flags = AI_PASSIVE;

  if (getaddrinfo(host_name, service.c_str(), &hints, &result) != 0)
  {
    LOG_ERROR("Failed to get address for %s:%d", host.c_str(), port);
    return false;
  }

  bool connected = false;
  // loop through the list of addresses untill we find one that's connectable
  for (struct addrinfo *p = result; p != nullptr; p = p->ai_next)
  {
    socket_fd_ = ::socket(p->ai_family, p->ai_socktype, p->ai_protocol);

    if (socket_fd_ != -1 && open(socket_fd_, p->ai_addr, p->ai_addrlen))
    {
      connected = true;
      break;
    }
  }

  freeaddrinfo(result);

  if (!connected)
  {
    state_ = SocketState::Invalid;
    LOG_ERROR("Connection setup failed for %s:%d", host.c_str(), port);
  }
  else
  {
    setOptions(socket_fd_);
    state_ = SocketState::Connected;
    LOG_INFO("Connection established for %s:%d", host.c_str(), port);
  }
  return connected;
}

bool TCPSocket::setSocketFD(int socket_fd)
{
  if (state_ == SocketState::Connected)
    return false;
  socket_fd_ = socket_fd;
  state_ = SocketState::Connected;
  return true;
}

void TCPSocket::close()
{
  if (state_ != SocketState::Connected)
    return;
  state_ = SocketState::Closed;
  ::shutdown(socket_fd_, SHUT_RDWR);
  socket_fd_ = -1;
}

std::string TCPSocket::getIP()
{
  sockaddr_in name;
  socklen_t len = sizeof(name);
  int res = ::getsockname(socket_fd_, (sockaddr *)&name, &len);

  if (res < 0)
  {
    LOG_ERROR("Could not get local IP");
    return std::string();
  }

  char buf[128];
  inet_ntop(AF_INET, &name.sin_addr, buf, sizeof(buf));
  return std::string(buf);
}

bool TCPSocket::read(uint8_t *buf, size_t buf_len, size_t &read)
{
  read = 0;

  if (state_ != SocketState::Connected)
    return false;

  ssize_t res = ::recv(socket_fd_, buf, buf_len, 0);

  if (res == 0)
  {
    state_ = SocketState::Disconnected;
    return false;
  }
  else if (res < 0)
    return false;

  read = static_cast<size_t>(res);
  return true;
}

bool TCPSocket::write(const uint8_t *buf, size_t buf_len, size_t &written)
{
  written = 0;

  if (state_ != SocketState::Connected)
    return false;

  size_t remaining = buf_len;

  // handle partial sends
  while (written < buf_len)
  {
    ssize_t sent = ::send(socket_fd_, buf + written, remaining, 0);

    if (sent <= 0)
      return false;

    written += sent;
    remaining -= sent;
  }

  return true;
}