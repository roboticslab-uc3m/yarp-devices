// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "DextraSerialControlboard.hpp"

#include <yarp/os/LogStream.h>
#include <yarp/os/Property.h>
#include <yarp/os/Value.h>

using namespace roboticslab;

// -----------------------------------------------------------------------------

bool DextraSerialControlboard::open(yarp::os::Searchable & config)
{
    yDebug() << "DextraSerialControlboard config:" << config.toString();

    std::string port = config.check("port", yarp::os::Value(DEFAULT_PORT), "serial port").asString();

    // Should match https://github.com/roboticslab-uc3m/Dextra/blob/master/Control/synapse.py
    // See also https://github.com/pyserial/pyserial/blob/master/serial/serialposix.py

    yarp::os::Property serialOptions;
    serialOptions.put("device", "serialport");
    serialOptions.put("comport", port);
    serialOptions.put("baudrate", 115200);
    serialOptions.put("paritymode", "NONE");
    serialOptions.put("databits", 8);

    yDebug() << "Serial device options:" << serialOptions.toString();

    if (!serialDevice.open(serialOptions))
    {
        yError() << "Unable to open" << serialOptions.find("device").asString() << "device";
        return false;
    }

    yarp::dev::ISerialDevice * iSerialDevice;

    if (!serialDevice.view(iSerialDevice))
    {
        yError() << "Unable to view iSerialDevice";
        return false;
    }

    raw.acquireSynapseHandle(new SerialSynapse(iSerialDevice));

    return true;
}

// -----------------------------------------------------------------------------

bool DextraSerialControlboard::close()
{
    raw.destroySynapse();
    serialDevice.close();
    return true;
}

// -----------------------------------------------------------------------------
