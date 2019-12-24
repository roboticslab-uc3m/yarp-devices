// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "TextilesHand.hpp"

#include <ColorDebug.h>

using namespace roboticslab;

// -----------------------------------------------------------------------------

bool TextilesHand::open(yarp::os::Searchable & config)
{
    CD_DEBUG("%s\n", config.toString().c_str());

    char serialport[13] = "/dev/ttyUSB0";
    int baudrate = B9600; // default
    char buf[256];
    int rc,n;

    fd = serialport_init(serialport, baudrate);

    if (!fd)
    {
        CD_ERROR("NULL fd, bye!\n");
        return false;
    }

    CD_SUCCESS("open(), fd: %d\n", fd);
    return true;
}

// -----------------------------------------------------------------------------

bool TextilesHand::close()
{
    return true;
}

// -----------------------------------------------------------------------------
