// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "WiimoteSensor.hpp"

#include <cstdlib>
#include <cstring>

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

    if (ret < 0)
    {
        CD_ERROR("Cannot create xwii_iface '%s': %d.\n", syspath, ret);
        std::free(syspath);
        return false;
    }

    CD_SUCCESS("Created xwii_iface '%s'.\n", syspath);
    std::free(syspath);

    ret = xwii_iface_open(iface, XWII_IFACE_CORE | XWII_IFACE_ACCEL);

    if (ret < 0)
    {
        CD_ERROR("Cannot open interface, did you launch with sudo? (%d).\n", ret);
        close();
        return false;
    }

    std::memset(fds, 0, sizeof(fds));

    fds[0].fd = 0;
    fds[0].events = POLLIN;

    fds[1].fd = xwii_iface_get_fd(iface);
    fds[1].events = POLLIN;

    fds_num = 2;

    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::WiimoteSensor::close()
{
    xwii_iface_unref(iface);
    return true;
}

// -----------------------------------------------------------------------------
