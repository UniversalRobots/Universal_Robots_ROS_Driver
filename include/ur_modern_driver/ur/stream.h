#pragma once
#include <atomic>
#include <netdb.h>
#include <string>
#include <sys/socket.h>
#include <sys/types.h>

/// Encapsulates a TCP socket
class URStream {
private:
    int _socket_fd = -1;
    std::string _host;
    int _port;

    std::atomic<bool> _initialized;
    std::atomic<bool> _stopping;

public:
    URStream(std::string& host, int port)
        : _host(host)
        , _port(port)
        , _initialized(false)
        , _stopping(false)
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