// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "DextraControlboardUSB.hpp"

#include <fcntl.h>    /* File control definitions */
#include <termios.h>  /* POSIX terminal control definitions */
#include <unistd.h>   /* UNIX standard function definitions */

#include <cerrno>
#include <cstdio>
#include <cstring>

#include <ColorDebug.h>

// -----------------------------------------------------------------------------

int roboticslab::DextraControlboardUSB::serialport_writebyte( int fd, uint8_t b)
{
    int n = ::write(fd, &b, 1);

    if (n == -1 && errno != EAGAIN)
    {
        return -1;
    }

    return 0;
}

// -----------------------------------------------------------------------------

int roboticslab::DextraControlboardUSB::serialport_write(int fd, const char* str)
{
    int len = std::strlen(str);
    int n = ::write(fd, str, len);

    if (n != len)
    {
        return -1;
    }

    return 0;
}

// -----------------------------------------------------------------------------

int roboticslab::DextraControlboardUSB::serialport_read_until(int fd, char* buf, char until)
{
    char b[1];
    int i=0;
    do
    {
        int n = ::read(fd, b, 1);  // read a char at a time
        if( n==-1) return -1;    // couldn't read
        if( n==0 )
        {
            ::usleep( 10 * 1000 ); // wait 10 msec try again
            continue;
        }
        buf[i] = b[0];
        i++;
    }
    while( b[0] != until );

    buf[i] = 0;  // null terminate the string
    return 0;
}

// -----------------------------------------------------------------------------

int roboticslab::DextraControlboardUSB::serialport_init(const char* serialport, int baud)
{
    CD_INFO("Opening port %s @ %d bps\n", serialport, baud);

    int fd = ::open(serialport, O_RDWR | O_NOCTTY | O_NONBLOCK);

    if (fd == -1)
    {
        std::perror("init_serialport: Unable to open port ");
        return -1;
    }

    struct termios toptions;

    if (::tcgetattr(fd, &toptions) < 0)
    {
        std::perror("init_serialport: Couldn't get term attributes");
        return -1;
    }

    // https://github.com/pyserial/pyserial/blob/f620b4727b67c968ba27a601ad76b57d2369f6d4/serial/serialposix.py

    // set up raw mode / no echo / binary
    toptions.c_cflag |= CLOCAL | CREAD;
    toptions.c_lflag &= ~(ICANON | ECHO | ECHOE | ECHOK | ECHONL | ECHOCTL | ECHOKE | ISIG | IEXTEN);
    toptions.c_oflag &= ~(OPOST | ONLCR | OCRNL);
    toptions.c_iflag &= ~(INLCR | IGNCR | ICRNL | IGNBRK | IUCLC | PARMRK);

    // setup baud rate
    speed_t brate = baud;
    ::cfsetispeed(&toptions, brate);
    ::cfsetospeed(&toptions, brate);

    // setup char len
    toptions.c_cflag &= ~CSIZE;
    toptions.c_cflag |= CS8;

    // setup stop bits
    toptions.c_cflag &= ~CSTOPB;

    // setup parity
    toptions.c_iflag &= ~(INPCK | ISTRIP);
    toptions.c_cflag &= ~(PARENB | PARODD | CMSPAR);

    // setup flow control
    toptions.c_iflag &= ~(IXON | IXOFF | IXANY);
    toptions.c_cflag |= CRTSCTS;

    toptions.c_cflag &= ~PARENB;
    toptions.c_cflag &= ~CSTOPB;
    toptions.c_cflag &= ~CSIZE;
    toptions.c_cflag |= CS8;
    // no flow control
    toptions.c_cflag &= ~CRTSCTS;

    toptions.c_cflag |= CREAD | CLOCAL;  // turn on READ & ignore ctrl lines
    toptions.c_iflag &= ~(IXON | IXOFF | IXANY); // turn off s/w flow ctrl

    toptions.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG); // make raw
    toptions.c_oflag &= ~OPOST; // make raw

    // timeout is done via select()
    toptions.c_cc[VMIN]  = 0;
    toptions.c_cc[VTIME] = 0;

    if( ::tcsetattr(fd, TCSANOW, &toptions) < 0)
    {
        std::perror("init_serialport: Couldn't set term attributes");
        return -1;
    }

    return fd;
}

// -----------------------------------------------------------------------------

void roboticslab::DextraControlboardUSB::serialport_close(int fd)
{
    ::close(fd);
}

// -----------------------------------------------------------------------------
