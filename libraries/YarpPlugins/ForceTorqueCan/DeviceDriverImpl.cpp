// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "ForceTorqueCan.hpp"

#include <yarp/os/LogStream.h>
#include <yarp/os/Property.h>
#include <yarp/os/ResourceFinder.h>

#include "LogComponent.hpp"

using namespace roboticslab;

// -----------------------------------------------------------------------------

bool ForceTorqueCan::open(yarp::os::Searchable & config)
{
    yarp::os::Property robotConfig;
    auto & rf = yarp::os::ResourceFinder::getResourceFinderSingleton();
    auto configPath = rf.findFileByName("config.ini"); // set YARP_ROBOT_NAME first

    if (configPath.empty() || !robotConfig.fromConfigFile(configPath))
    {
        yCError(FTC) << "Robot config file not found or insufficient permissions:" << configPath;
        return false;
    }

    auto canBusGroupName = config.check("canBus", yarp::os::Value("default"), "CAN bus device group").asString();
    const auto & canBusGroup = robotConfig.findGroup(canBusGroupName);

    if (canBusGroup.isNull())
    {
        yCError(FTC) << "Missing CAN bus device group:" << canBusGroupName;
        return false;
    }

    yarp::os::Property canBusOptions;
    canBusOptions.setMonitor(config.getMonitor(), canBusGroupName.c_str());
    canBusOptions.fromString(canBusGroup.toString());
    canBusOptions.put("blockingMode", false); // enforce non-blocking mode
    canBusOptions.put("allowPermissive", false); // always check usage requirements

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
