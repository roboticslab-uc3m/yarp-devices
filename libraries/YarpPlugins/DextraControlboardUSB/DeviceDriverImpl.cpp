// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "DextraControlboardUSB.hpp"

#include <termios.h>

#include <ColorDebug.h>

// -----------------------------------------------------------------------------
bool roboticslab::DextraControlboardUSB::open(yarp::os::Searchable& config)
{
    std::string port = config.check("port", yarp::os::Value(DEFAULT_PORT), "setial port").asString();

    // Should match https://github.com/roboticslab-uc3m/Dextra/blob/master/Control/synapse.py
    const int baudrate = B115200;

    fd = serialport_init(port.c_str(), baudrate);

    if (fd <= 0)
    {
        CD_ERROR("Could not open %s (fd = %d <= 0). Bye!\n", port.c_str(), fd);
        return false;
    }

    CD_SUCCESS("Opened %s (fd: %d)\n", port.c_str(), fd);

    return true;
}

// -----------------------------------------------------------------------------
bool roboticslab::DextraControlboardUSB::close()
{
    CD_INFO("\n");
    return true;
}

// -----------------------------------------------------------------------------
