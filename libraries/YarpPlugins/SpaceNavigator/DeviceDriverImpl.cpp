// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "SpaceNavigator.hpp"

#include <yarp/os/LogStream.h>

#include "LogComponent.hpp"

// -----------------------------------------------------------------------------

bool SpaceNavigator::open(yarp::os::Searchable & config)
{
    if (!parseParams(config))
    {
        yCError(SPNAV) << "Failed to parse parameters";
        return false;
    }

    if (m_deadband < 0.0 || m_deadband > 1.0)
    {
        yCError(SPNAV) << "Invalid deadband value (must be in [0,1]):" << m_deadband;
        return false;
    }

    if (::spnav_open() == -1)
    {
        yCError(SPNAV) << "Failed to connect to spacenavd";
        return false;
    }

#ifdef _SPNAV_NEW_INTERFACE
    if (char buf[256]; ::spnav_dev_name(buf, sizeof(buf)) != -1)
    {
        yCInfo(SPNAV) << "Device:" << std::string(buf);
    }

    if (char buf[256]; ::spnav_dev_path(buf, sizeof(buf)) != -1)
    {
        yCInfo(SPNAV) << "Path:" << std::string(buf);
    }

    auto buttons = ::spnav_dev_buttons();
    yCInfo(SPNAV) << "Buttons:" << buttons;

    auto axes = ::spnav_dev_axes();
    yCInfo(SPNAV) << "Axes:" << axes;

    if (unsigned int vendor, product; ::spnav_dev_usbid(&vendor, &product) != -1)
    {
        yCInfo(SPNAV, "USB ID: %04x:%04x", vendor, product);
    }

    if (int type; (type = ::spnav_dev_type()) != -1)
    {
        yCInfo(SPNAV, "Device type: 0x%03X", type);
    }

    if (::spnav_evmask(SPNAV_EVMASK_INPUT) == -1)
    {
        yCError(SPNAV) << "Failed to set event mask";
        return false;
    }
#endif

    return yarp::os::Thread::start();
}

// -----------------------------------------------------------------------------

bool SpaceNavigator::close()
{
    return yarp::os::Thread::stop() & ::spnav_close() != -1;
}

// -----------------------------------------------------------------------------
