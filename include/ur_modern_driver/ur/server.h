#pragma once
#include <netdb.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <cstdlib>
#include <mutex>
#include <atomic>
#include <string>
#include "ur_modern_driver/ur/stream.h"

class URServer
{
private:
  int socket_fd_ = -1;

public:
  URServer(int port);
  URStream accept();
};