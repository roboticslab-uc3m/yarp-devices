// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "TextilesHand.hpp"

#include <yarp/os/Property.h>

#include <ColorDebug.h>

using namespace roboticslab;

// -----------------------------------------------------------------------------

bool TextilesHand::open(yarp::os::Searchable & config)
{
    CD_DEBUG("%s\n", config.toString().c_str());

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
        CD_ERROR("Unable to open serial device.\n");
        return false;
    }

    if (!serialDevice.view(iSerialDevice))
    {
        CD_ERROR("Unable to view iSerialDevice.\n");
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
