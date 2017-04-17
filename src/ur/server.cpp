#include <cstring>
#include <netinet/tcp.h>
#include <unistd.h>
#include "ur_modern_driver/log.h"
#include "ur_modern_driver/ur/server.h"

URServer::URServer(int port)
{
  std::string service = std::to_string(port);
  struct addrinfo hints, *result;
  std::memset(&hints, 0, sizeof(hints));

  hints.ai_family = AF_UNSPEC;
  hints.ai_socktype = SOCK_STREAM;
  hints.ai_flags = AI_PASSIVE;

  if (getaddrinfo(nullptr, service.c_str(), &hints, &result) != 0)
  {
    LOG_ERROR("Failed to setup recieving server");
    return;
  }

    // loop through the list of addresses untill we find one that's connectable
  for (struct addrinfo* p = result; p != nullptr; p = p->ai_next)
  {
    socket_fd_ = socket(p->ai_family, p->ai_socktype, p->ai_protocol);

    if (socket_fd_ == -1)  // socket error?
      continue;

    if (bind(socket_fd_, p->ai_addr, p->ai_addrlen) != 0)
      continue;

    // disable Nagle's algorithm to ensure we sent packets as fast as possible
    int flag = 1;
    setsockopt(socket_fd_, IPPROTO_TCP, TCP_NODELAY, &flag, sizeof(flag));
    LOG_INFO("Server awaiting connection");
    return;
  }
  
  LOG_ERROR("Failed to setup recieving server");
  std::exit(EXIT_FAILURE);
}

URStream URServer::accept()
{
  struct sockaddr addr; 
  socklen_t addr_len;
  int client_fd = ::accept(socket_fd_, &addr, &addr_len);
  return URStream(client_fd);
}