// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "Jr3Pci.hpp"

#include <sys/ioctl.h>

#include <yarp/os/LogStream.h>

#include "LogComponent.hpp"

// -----------------------------------------------------------------------------

bool Jr3Pci::calibrateSensor()
{
    yCInfo(JR3P) << "Calibrating sensor..."; // = set to zero

    return calibrateChannel(0) &
           calibrateChannel(1) &
           calibrateChannel(2) &
           calibrateChannel(3);
}

// -----------------------------------------------------------------------------

bool Jr3Pci::calibrateChannel(int ch)
{
    int ok;

    switch(ch)
    {
    case 0:
        ok = ::ioctl(fd, IOCTL0_JR3_ZEROOFFS);
        break;
    case 1:
        ok = ::ioctl(fd, IOCTL1_JR3_ZEROOFFS);
        break;
    case 2:
        ok = ::ioctl(fd, IOCTL2_JR3_ZEROOFFS);
        break;
    case 3:
        ok = ::ioctl(fd, IOCTL3_JR3_ZEROOFFS);
        break;
    default:
        yCError(JR3P) << "Illegal channel" << ch;
        ok = -1;
        break;
    }

    if (ok == -1)
    {
        yCError(JR3P) << "ioctl() on calibrate channel" << ch << "failed";
        return false;
    }

    return true;
}

// -----------------------------------------------------------------------------
