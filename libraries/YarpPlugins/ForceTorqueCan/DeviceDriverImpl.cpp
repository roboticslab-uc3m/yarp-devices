// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "ForceTorqueCan.hpp"

#include <yarp/os/LogStream.h>
#include <yarp/os/Property.h>

#include "LogComponent.hpp"

using namespace roboticslab;

constexpr auto DEFAULT_CAN_BUS_DEVICE = "CanBusSocket";

// -----------------------------------------------------------------------------

bool ForceTorqueCan::open(yarp::os::Searchable & config)
{
    auto canBusDevice = config.check("canBus", yarp::os::Value(DEFAULT_CAN_BUS_DEVICE), "CAN bus device name").asString();

    yarp::os::Property canBusOptions;
    canBusOptions.setMonitor(config.getMonitor(), canBusDevice.c_str());

    if (!canBus.open(canBusOptions))
    {
        yCError(FTC) << "Failed to open CAN device";
        return false;
    }

    if (!canBus.view(iCanBus) || !canBus.view(iCanBufferFactory))
    {
        yCError(FTC) << "Failed to acquire CAN device interfaces";
        return false;
    }

    return yarp::os::Thread::start();
}

// -----------------------------------------------------------------------------

bool ForceTorqueCan::close()
{
    bool ok = yarp::os::Thread::stop();
    ok &= canBus.close();
    return ok;
}

// -----------------------------------------------------------------------------
