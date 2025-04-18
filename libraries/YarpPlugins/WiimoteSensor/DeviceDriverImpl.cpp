// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "WiimoteSensor.hpp"

#include <cstdlib>

#include <yarp/os/LogStream.h>

#include "LogComponent.hpp"

// -----------------------------------------------------------------------------

bool WiimoteSensor::open(yarp::os::Searchable& config)
{
    if (!parseParams(config))
    {
        yCError(WII) << "Cannot parse parameters";
        return false;
    }

    char * syspath = getDevicePath(m_deviceId);

    if (syspath == nullptr)
    {
        yCError(WII) << "Cannot find device with number" << m_deviceId;
        return false;
    }

    int ret = xwii_iface_new(&iface, syspath);

    if (ret < 0)
    {
        yCError(WII, "Cannot create xwii_iface %s: %d", syspath, ret);
        std::free(syspath);
        return false;
    }

    yCInfo(WII) << "Created xwii_iface" << syspath;
    std::free(syspath);

    ret = xwii_iface_open(iface, XWII_IFACE_CORE | XWII_IFACE_ACCEL);

    if (ret < 0)
    {
        yCError(WII) << "Cannot open interface, did you launch with sudo?" << ret;
        return false;
    }

    dispatcherThread.setInterfacePointer(iface);
    return dispatcherThread.start();
}

// -----------------------------------------------------------------------------

bool WiimoteSensor::close()
{
    dispatcherThread.stop();

    if (iface)
    {
        xwii_iface_unref(iface);
        iface = nullptr;
    }

    return true;
}

// -----------------------------------------------------------------------------
