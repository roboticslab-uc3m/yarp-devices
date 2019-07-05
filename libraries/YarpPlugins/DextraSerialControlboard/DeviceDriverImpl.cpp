// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "DextraSerialControlboard.hpp"

#include <cstring>

#include <yarp/os/Property.h>
#include <yarp/os/Value.h>

#include <ColorDebug.h>

// -----------------------------------------------------------------------------

bool roboticslab::DextraSerialControlboard::open(yarp::os::Searchable& config)
{
    std::string port = config.check("port", yarp::os::Value(DEFAULT_PORT), "serial port").asString();

    // Should match https://github.com/roboticslab-uc3m/Dextra/blob/master/Control/synapse.py
    // See also https://github.com/pyserial/pyserial/blob/master/serial/serialposix.py

    yarp::os::Property serialOptions;
    serialOptions.put("device", "serialport");
    serialOptions.put("comport", port);
    serialOptions.put("baudrate", 115200);
    serialOptions.put("paritymode", "NONE");
    serialOptions.put("databits", 8);

    CD_DEBUG("Serial device options: %s\n", serialOptions.toString().c_str());

    if (!serialDevice.open(serialOptions))
    {
        CD_ERROR("Unable to open %s device.\n", serialOptions.find("device").asString().c_str());
        return false;
    }

    yarp::dev::ISerialDevice * iSerialDevice;

    if (!serialDevice.view(iSerialDevice))
    {
        CD_ERROR("Unable to view iSerialDevice.\n");
        return true;
    }

    std::memset(setpoints, 0, sizeof(setpoints));

    synapse = new Synapse(iSerialDevice);

    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::DextraSerialControlboard::close()
{
    delete synapse;
    serialDevice.close();
    return true;
}

// -----------------------------------------------------------------------------
