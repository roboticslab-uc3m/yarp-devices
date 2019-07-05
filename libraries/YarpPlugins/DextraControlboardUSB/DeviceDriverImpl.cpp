// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "DextraControlboardUSB.hpp"

#include <cstring>

#include <yarp/os/Property.h>

#include <ColorDebug.h>

// -----------------------------------------------------------------------------

bool roboticslab::DextraControlboardUSB::open(yarp::os::Searchable& config)
{
    std::string port = config.check("port", yarp::os::Value(DEFAULT_PORT), "serial port").asString();

    // Should match https://github.com/roboticslab-uc3m/Dextra/blob/master/Control/synapse.py
    yarp::os::Property serialOptions;
    serialOptions.put("device", "serialport");
    serialOptions.put("comport", port);
    serialOptions.put("baudrate", 115200);

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

    synapse = new Synapse(iSerialDevice);

    std::memset(setpoints, 0, Synapse::DATA_POINTS);

    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::DextraControlboardUSB::close()
{
    delete synapse;
    serialDevice.close();
    return true;
}

// -----------------------------------------------------------------------------
