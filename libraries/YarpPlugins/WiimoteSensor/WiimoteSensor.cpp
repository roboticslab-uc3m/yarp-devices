// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "WiimoteSensor.hpp"

#include <cstdlib>

#include "ColorDebug.h"

// -----------------------------------------------------------------------------

// Copy-paste from xwiishow.c.
char * roboticslab::WiimoteSensor::getDevicePath(int id)
{
    struct xwii_monitor * monitor;
    char * ent;
    int i = 0;

    monitor = xwii_monitor_new(false, false);

    if (monitor == NULL)
    {
        CD_ERROR("Cannot create monitor.\n");
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
