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

    const auto * canBuses = config.find("canBuses").asList();

    if (!canBuses)
    {
        yCError(FTC) << "Missing key \"canBuses\" or not a list";
        return false;
    }

    for (int i = 0; i < canBuses->size(); i++)
    {
        auto canBus = canBuses->get(i).asString();

        if (!config.check(canBus))
        {
            yCError(FTC) << "Missing CAN bus key:" << canBus;
            return false;
        }

        const auto & canBusGroup = robotConfig.findGroup(canBus);

        if (canBusGroup.isNull())
        {
            yCError(FTC) << "Missing CAN bus device group:" << canBus;
            return false;
        }

        auto * canBusDevice = new yarp::dev::PolyDriver;
        canBusDevices.push_back(canBusDevice);

        yarp::os::Property canBusOptions;
        canBusOptions.setMonitor(config.getMonitor(), canBus.c_str());
        canBusOptions.fromString(canBusGroup.toString());
        canBusOptions.put("blockingMode", false); // enforce non-blocking mode
        canBusOptions.put("allowPermissive", false); // always check usage requirements

        if (!canBusDevice->open(canBusOptions))
        {
            yCError(FTC) << "Failed to open CAN device:" << canBus;
            return false;
        }

        yarp::dev::ICanBus * iCanBus;
        yarp::dev::ICanBufferFactory * iCanBufferFactory;

        if (!canBusDevice->view(iCanBus) || !canBusDevice->view(iCanBufferFactory))
        {
            yCError(FTC) << "Failed to acquire CAN device interfaces:" << canBus;
            return false;
        }

        auto * canReadThread = new CanReadThread(iCanBus, iCanBufferFactory);
        canReadThreads.push_back(canReadThread);

        if (!canReadThread->start())
        {
            yCError(FTC) << "Failed to start CAN read thread:" << canBus;
            return false;
        }

        yCInfo(FTC) << "Started CAN read thread:" << canBus;
    }

    return true;
}

// -----------------------------------------------------------------------------

bool ForceTorqueCan::close()
{
    bool ok = true;

    for (auto * canReadThread : canReadThreads)
    {
        ok &= canReadThread->stop();
        delete canReadThread;
    }

    canReadThreads.clear();

    for (auto * canBusDevice : canBusDevices)
    {
        ok &= canBusDevice->close();
        delete canBusDevice;
    }

    canBusDevices.clear();

    return ok;
}

// -----------------------------------------------------------------------------
