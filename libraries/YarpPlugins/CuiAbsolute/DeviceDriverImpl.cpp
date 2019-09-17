// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "CuiAbsolute.hpp"

#include <cmath>

#include <yarp/os/Time.h>

#include <ColorDebug.h>

using namespace roboticslab;

// -----------------------------------------------------------------------------

bool CuiAbsolute::open(yarp::os::Searchable& config)
{
    CD_DEBUG("%s.\n", config.toString().c_str());

    canId = config.check("canId", yarp::os::Value(0), "CAN bus ID").asInt32();
    reverse = config.check("reverse", yarp::os::Value(false), "reverse").asBool();
    cuiTimeout = config.check("cuiTimeout", yarp::os::Value(0), "CUI timeout (seconds)").asInt32();

    if (cuiTimeout <= 0.0)
    {
        CD_ERROR("Illegal CUI timeout value: %f.\n", cuiTimeout);
        return false;
    }

    CD_SUCCESS("Created CuiAbsolute with canId %d.\n", canId);
    return true;
}

// -----------------------------------------------------------------------------

bool CuiAbsolute::close()
{
    CD_INFO("Stopping Cui Absolute PIC (ID: %d)\n", canId);

    if (!stopPublishingMessages())
    {
        return false;
    }

    yarp::os::Time::delay(0.5);

    const double start = yarp::os::Time::now();
    const static double margin = 0.1; // seconds
    const static double timeOut = 1.0; // seconds

    double lastRead;
    double now;

    do
    {
        now = yarp::os::Time::now();

        mutex.lock();
        lastRead = encoderTimestamp;
        mutex.unlock();

        if (now - lastRead > margin)
        {
            break;
        }

        CD_WARNING("Resending stop message to Cui Absolute PIC (ID: %d)\n", canId);
        stopPublishingMessages();

        yarp::os::Time::delay(margin);
    }
    while (now - start < timeOut);

    CD_SUCCESS("Time out passed and CuiAbsolute ID %d was stopped successfully\n", canId);
    return true;
}

// -----------------------------------------------------------------------------
