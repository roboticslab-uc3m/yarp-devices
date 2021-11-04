// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "TextilesHand.hpp"

#include <yarp/conf/version.h>

#include <yarp/os/LogStream.h>
#include <yarp/os/Property.h>

#include "LogComponent.hpp"

using namespace roboticslab;

// -----------------------------------------------------------------------------

bool TextilesHand::open(yarp::os::Searchable & config)
{
#if YARP_VERSION_MINOR < 6
    yCDebug(TXT) << "Config:" << config.toString();
#endif

    std::string port = config.check("port", yarp::os::Value(DEFAULT_PORT), "serial port").asString();

    // check firmware/TextilesHand/pwmServer/pwmServer.ino
    // ...and https://www.arduino.cc/reference/en/language/functions/communication/serial/begin/
    // ...with: "The default is 8 data bits, no parity, one stop bit."

    yarp::os::Property serialOptions({
        {"device", yarp::os::Value("serialport")},
        {"comport", yarp::os::Value(port)},
        {"baudrate", yarp::os::Value(9600)},
        {"databits", yarp::os::Value(8)},
        {"paritymode", yarp::os::Value("NONE")},
        {"stopbits", yarp::os::Value(1)}
    });

    if (!serialDevice.open(serialOptions))
    {
        yCError(TXT) << "Unable to open serial device";
        return false;
    }

    if (!serialDevice.view(iSerialDevice))
    {
        yCError(TXT) << "Unable to view iSerialDevice";
        return false;
    }

    return true;
}

// -----------------------------------------------------------------------------

bool TextilesHand::close()
{
    return serialDevice.close();
}

// -----------------------------------------------------------------------------
