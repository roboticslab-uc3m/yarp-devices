// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "WiimoteSensor.hpp"

#include <cstdlib>

#include <yarp/os/Value.h>

#include "ColorDebug.hpp"

// -----------------------------------------------------------------------------

bool roboticslab::WiimoteSensor::open(yarp::os::Searchable& config)
{
    int deviceId = config.check("deviceId", yarp::os::Value(DEFAULT_DEVICE), "Wiimote device number").asInt();

    char * syspath = getDevicePath(deviceId);

    if (syspath == NULL)
    {
        CD_ERROR("Cannot find device with number %d.\n", deviceId);
        return false;
    }

    int ret = xwii_iface_new(&iface, syspath);

    std::free(syspath);

    if (ret < 0)
    {
        CD_ERROR("Cannot create xwii_iface '%s' (error: %d).\n", syspath, ret);
        return false;
    }

    ret = xwii_iface_open(iface, XWII_IFACE_CORE | XWII_IFACE_ACCEL);

    if (ret < 0)
    {
        CD_ERROR("Cannot open interface: %d.\n", ret);
        close();
        return false;
    }

    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::WiimoteSensor::close()
{
    xwii_iface_unref(iface);
    return true;
}

// -----------------------------------------------------------------------------
