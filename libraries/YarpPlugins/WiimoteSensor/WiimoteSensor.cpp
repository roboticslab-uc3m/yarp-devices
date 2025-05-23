// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "WiimoteSensor.hpp"

#include <cstdlib>

#include "yarp/os/LogStream.h"

#include "LogComponent.hpp"

// -----------------------------------------------------------------------------

// Copy-paste from xwiishow.c.
char * WiimoteSensor::getDevicePath(int id)
{
    struct xwii_monitor * monitor;
    char * ent;
    int i = 0;

    monitor = xwii_monitor_new(false, false);

    if (monitor == NULL)
    {
        yCError(WII) << "Cannot create monitor";
        return NULL;
    }

    while ((ent = xwii_monitor_poll(monitor)))
    {
        if (++i == id)
        {
            break;
        }

        std::free(ent);
    }

    xwii_monitor_unref(monitor);

    return ent;
}

// -----------------------------------------------------------------------------
