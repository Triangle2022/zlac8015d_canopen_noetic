#pragma once

#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cstring>
#include <stdexcept>
#include <string>

namespace zlac8015d
{

class SocketCan
{
public:
  SocketCan() = default;

  ~SocketCan()
  {
    close();
  }

  SocketCan(const SocketCan&) = delete;
  SocketCan& operator=(const SocketCan&) = delete;

  void open(const std::string& interface_name)
  {
    close();

    socket_fd_ = ::socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (socket_fd_ < 0)
    {
      throw std::runtime_error("failed to create CAN socket");
    }

    struct ifreq ifr;
    std::memset(&ifr, 0, sizeof(ifr));
    std::strncpy(ifr.ifr_name, interface_name.c_str(), IFNAMSIZ - 1);

    if (::ioctl(socket_fd_, SIOCGIFINDEX, &ifr) < 0)
    {
      close();
      throw std::runtime_error("failed to resolve CAN interface: " + interface_name);
    }

    struct sockaddr_can addr;
    std::memset(&addr, 0, sizeof(addr));
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    if (::bind(socket_fd_, reinterpret_cast<struct sockaddr*>(&addr), sizeof(addr)) < 0)
    {
      close();
      throw std::runtime_error("failed to bind CAN interface: " + interface_name);
    }
  }

  void close()
  {
    if (socket_fd_ >= 0)
    {
      ::close(socket_fd_);
      socket_fd_ = -1;
    }
  }

  void send(const can_frame& frame)
  {
    const auto written = ::write(socket_fd_, &frame, sizeof(frame));
    if (written != sizeof(frame))
    {
      throw std::runtime_error("failed to write CAN frame");
    }
  }

  bool receive(can_frame& frame, const int timeout_ms)
  {
    struct pollfd fds;
    fds.fd = socket_fd_;
    fds.events = POLLIN;
    fds.revents = 0;

    const int ready = ::poll(&fds, 1, timeout_ms);
    if (ready <= 0)
    {
      return false;
    }

    const auto bytes = ::read(socket_fd_, &frame, sizeof(frame));
    return bytes == sizeof(frame);
  }

  bool isOpen() const
  {
    return socket_fd_ >= 0;
  }

private:
  int socket_fd_{-1};
};

}  // namespace zlac8015d
