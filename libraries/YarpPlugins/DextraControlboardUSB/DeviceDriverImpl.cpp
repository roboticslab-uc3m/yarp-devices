// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "DextraControlboardUSB.hpp"

#include <termios.h>

#include <ColorDebug.h>

// -----------------------------------------------------------------------------
bool roboticslab::DextraControlboardUSB::open(yarp::os::Searchable& config)
{
    char serialport[13] = "/dev/ttyACM0";  // Was /dev/ttyUSB0
    int baudrate = B115200;  // Should match https://github.com/Alvipe/Dextra/blob/master/Control/DextraControl.py
    char buf[256];
    int rc,n;

    fd = serialport_init(serialport, baudrate);

    if ( fd <= 0 )
    {
        CD_ERROR("Could not open %s (fd = %d <= 0). Bye!\n",serialport,fd);
        return false;
    }

    CD_SUCCESS("Opened %s (fd: %d)\n",serialport,fd);

    return true;
}

// -----------------------------------------------------------------------------
bool roboticslab::DextraControlboardUSB::close()
{
    CD_INFO("\n");
    return true;
}

// -----------------------------------------------------------------------------
