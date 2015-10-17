#include <provider_dvl/base/IOStream.hpp>
#include <provider_dvl/base/Exceptions.hpp>
#include <ros/ros.h>

#include <sys/types.h> 
#include <sys/stat.h> 
#include <unistd.h>
#include <fcntl.h>

#include <errno.h>
#include <iostream>

using namespace iodrivers_base;

IOStream::~IOStream() {}
int IOStream::getFileDescriptor() const { return FDStream::INVALID_FD; }

FDStream::FDStream(int fd, bool auto_close)
    : m_auto_close(auto_close)
    , m_fd(fd)

{
    if (setNonBlockingFlag(fd))
    {
      ROS_WARN( "FD given to Driver::setFileDescriptor is set as blocking, setting the NONBLOCK flag");
    }
}
FDStream::~FDStream()
{
    if (m_auto_close)
        ::close(m_fd);
}
void FDStream::waitRead(base::Time const& timeout)
{
    fd_set set;
    FD_ZERO(&set);
    FD_SET(m_fd, &set);

    timeval timeout_spec = { static_cast<time_t>(timeout.toSeconds()), timeout.toMicroseconds() % 1000000 };
    int ret = select(m_fd + 1, &set, NULL, NULL, &timeout_spec);
    if (ret < 0 && errno != EINTR)
        throw UnixError("waitRead(): error in select()");
    else if (ret == 0)
        throw TimeoutError(TimeoutError::NONE, "waitRead(): timeout");
}
void FDStream::waitWrite(base::Time const& timeout)
{
    fd_set set;
    FD_ZERO(&set);
    FD_SET(m_fd, &set);

    timeval timeout_spec = { static_cast<time_t>(timeout.toSeconds()), timeout.toMicroseconds() % 1000000 };
    int ret = select(m_fd + 1, NULL, &set, NULL, &timeout_spec);
    if (ret < 0 && errno != EINTR)
        throw UnixError("waitWrite(): error in select()");
    else if (ret == 0)
        throw TimeoutError(TimeoutError::NONE, "waitWrite(): timeout");
}
size_t FDStream::read(uint8_t* buffer, size_t buffer_size)
{
    int c = ::read(m_fd, buffer, buffer_size);
    if (c > 0)
        return c;
    else if (c == 0)
        return 0;
    else
    {
        if (errno == EAGAIN)
            return 0;
        throw UnixError("readPacket(): error reading the file descriptor");
    }
}
size_t FDStream::write(uint8_t const* buffer, size_t buffer_size)
{
    int c = ::write(m_fd, buffer, buffer_size);
    if (c == -1 && errno != EAGAIN && errno != ENOBUFS)
        throw UnixError("writePacket(): error during write");
    if (c == -1)
        return 0;
    return c;
}
void FDStream::clear()
{
}
bool FDStream::setNonBlockingFlag(int fd)
{
    long fd_flags = fcntl(fd, F_GETFL);
    if (!(fd_flags & O_NONBLOCK))
    {
        if (fcntl(fd, F_SETFL, fd_flags | O_NONBLOCK) == -1)
            throw UnixError("cannot set the O_NONBLOCK flag");
        return true;
    }
    return false;
}
int FDStream::getFileDescriptor() const { return m_fd; }

UDPServerStream::UDPServerStream(int fd, bool auto_close)
  : FDStream(fd,auto_close)
{
  m_s_len = sizeof(m_si_other);
}
 
size_t UDPServerStream::read(uint8_t* buffer, size_t buffer_size)
{
  size_t ret = recvfrom(m_fd, buffer, buffer_size, 0, &m_si_other, &m_s_len);
  if (ret >= 0){
     return ret;
  }
  else
  {
    if (errno == EAGAIN){
      return 0;
    }
    throw UnixError("readPacket(): error reading the file descriptor");
  }
}

size_t UDPServerStream::write(uint8_t const* buffer, size_t buffer_size)
{
  ssize_t ret = sendto(m_fd, buffer, buffer_size, 0, &m_si_other, m_s_len);
  if (ret == -1 && errno != EAGAIN && errno != ENOBUFS){
    throw UnixError("UDPServerStream: writePacket(): error during write");
  }
  if (ret == -1){
    return 0;
  }
  return ret;
}
