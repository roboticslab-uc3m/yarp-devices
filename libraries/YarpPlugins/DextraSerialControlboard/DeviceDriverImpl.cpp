// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "DextraSerialControlboard.hpp"

#include <yarp/conf/version.h>

#include <yarp/os/LogComponent.h>
#include <yarp/os/LogStream.h>
#include <yarp/os/Property.h>
#include <yarp/os/Value.h>

using namespace roboticslab;

namespace
{
    YARP_LOG_COMPONENT(DEXTRA, "rl.DextraSerialControlboard")
}

constexpr auto DEFAULT_PORT = "/dev/ttyACM0"; // also /dev/ttyUSB0

// -----------------------------------------------------------------------------

bool DextraSerialControlboard::open(yarp::os::Searchable & config)
{
#if !defined(YARP_VERSION_COMPARE) // < 3.6.0
    yCDebug(DEXTRA) << "Config:" << config.toString();
#endif

    std::string port = config.check("port", yarp::os::Value(DEFAULT_PORT), "serial port").asString();

    // Should match https://github.com/roboticslab-uc3m/Dextra/blob/master/Control/synapse.py
    // See also https://github.com/pyserial/pyserial/blob/master/serial/serialposix.py

    yarp::os::Property serialOptions {
        {"device", yarp::os::Value("serialport")},
        {"comport", yarp::os::Value(port)},
        {"baudrate", yarp::os::Value(115200)},
        {"paritymode", yarp::os::Value("NONE")},
        {"databits", yarp::os::Value(8)},
    };

    yCDebug(DEXTRA) << "Serial device options:" << serialOptions.toString();

    if (!serialDevice.open(serialOptions))
    {
        yCError(DEXTRA) << "Unable to open" << serialOptions.find("device").asString() << "device";
        return false;
    }

    yarp::dev::ISerialDevice * iSerialDevice;

    if (!serialDevice.view(iSerialDevice))
    {
        yCError(DEXTRA) << "Unable to view iSerialDevice";
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
