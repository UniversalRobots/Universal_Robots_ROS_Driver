#include <cstring>
#include <endian.h>
#include <netinet/tcp.h>
#include <unistd.h>

#include "ur_modern_driver/log.h"
#include "ur_modern_driver/ur/stream.h"

bool URStream::connect()
{
    if (_initialized)
        return false;

    LOG_INFO("Connecting to UR @ %s:%d", _host.c_str(), _port);

    //gethostbyname() is deprecated so use getadderinfo() as described in:
    //http://www.beej.us/guide/bgnet/output/html/multipage/syscalls.html#getaddrinfo

    std::string service = std::to_string(_port);
    struct addrinfo hints, *result;
    std::memset(&hints, 0, sizeof(hints));

    hints.ai_family = AF_UNSPEC;
    hints.ai_socktype = SOCK_STREAM;
    hints.ai_flags = AI_PASSIVE;

    if (getaddrinfo(_host.c_str(), service.c_str(), &hints, &result) != 0) {
        LOG_ERROR("Failed to get host name");
        return false;
    }

    //loop through the list of addresses untill we find one that's connectable
    for (struct addrinfo* p = result; p != nullptr; p = p->ai_next) {
        _socket_fd = socket(p->ai_family, p->ai_socktype, p->ai_protocol);

        if (_socket_fd == -1) //socket error?
            continue;

        if (::connect(_socket_fd, p->ai_addr, p->ai_addrlen) != 0) {
            if (_stopping)
                break;
            else
                continue; //try next addrinfo if connect fails
        }

        //disable Nagle's algorithm to ensure we sent packets as fast as possible
        int flag = 1;
        setsockopt(_socket_fd, IPPROTO_TCP, TCP_NODELAY, &flag, sizeof(flag));
        _initialized = true;
        LOG_INFO("Connection successfully established");
        break;
    }

    freeaddrinfo(result);
    if (!_initialized)
        LOG_ERROR("Connection failed");

    return _initialized;
}

void URStream::disconnect()
{
    if (!_initialized || _stopping)
        return;

    LOG_INFO("Disconnecting from %s:%d", _host.c_str(), _port);

    _stopping = true;
    close(_socket_fd);
    _initialized = false;
}

ssize_t URStream::send(uint8_t* buf, size_t buf_len)
{
    if (!_initialized)
        return -1;
    if (_stopping)
        return 0;

    size_t total = 0;
    size_t remaining = buf_len;

    //TODO: handle reconnect?
    //handle partial sends
    while (total < buf_len) {
        ssize_t sent = ::send(_socket_fd, buf + total, remaining, 0);
        if (sent <= 0)
            return _stopping ? 0 : sent;
        total += sent;
        remaining -= sent;
    }

    return total;
}

ssize_t URStream::receive(uint8_t* buf, size_t buf_len)
{
    if (!_initialized)
        return -1;
    if (_stopping)
        return 0;

    size_t remainder = sizeof(int32_t);
    uint8_t* buf_pos = buf;
    bool initial = true;

    do {
        ssize_t read = recv(_socket_fd, buf_pos, remainder, 0);
        if (read <= 0) //failed reading from socket
            return _stopping ? 0 : read;

        if (initial) {
            remainder = be32toh(*(reinterpret_cast<int32_t*>(buf)));
            if (remainder >= (buf_len - sizeof(int32_t))) {
                LOG_ERROR("Packet size %d is larger than buffer %d, discarding.", remainder, buf_len);
                return -1;
            }
            initial = false;
        }

        buf_pos += read;
        remainder -= read;
    } while (remainder > 0);

    return buf_pos - buf;
}