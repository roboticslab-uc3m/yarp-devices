// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "CuiAbsolute.hpp"

#include <cmath>

#include <yarp/os/Time.h>

#include <ColorDebug.h>

// -----------------------------------------------------------------------------
bool roboticslab::CuiAbsolute::open(yarp::os::Searchable& config)
{
    canId = config.check("canId", yarp::os::Value(0), "can bus ID").asInt32();
    tr = config.check("tr", yarp::os::Value(1), "reduction").asInt32();
    cuiTimeout = config.check("cuiTimeout", yarp::os::Value(0), "CUI timeout (seconds)").asInt32();
    encoder = std::sqrt(-1); // NaN \todo{Investigate, debug and document the dangers of this use of NaN.}

    if (cuiTimeout <= 0.0)
    {
        CD_ERROR("Illegal CUI timeout value: %f.\n", cuiTimeout);
        return false;
    }

    CD_SUCCESS("Created CuiAbsolute with canId %d and tr %f, and all local parameters set to 0.\n", canId, tr);
    return true;
}

// -----------------------------------------------------------------------------
bool roboticslab::CuiAbsolute::close()
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

        encoderReady.wait();
        lastRead = encoderTimestamp;
        encoderReady.post();

        if (now - lastRead > margin)
        {
            break;
        }

        CD_WARNING("Resending stop message to Cui Absolute PIC (ID: %d)\n", canId);
        stopPublishingMessages();

        yarp::os::Time::delay(margin);
    }
    while (now - start < timeOut);

    CD_SUCCESS("Time out passed and CuiAbsolute ID (%d) was stopped successfully\n", canId);
    return true;
}

// -----------------------------------------------------------------------------
