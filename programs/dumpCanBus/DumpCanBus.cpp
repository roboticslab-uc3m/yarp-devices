// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "DumpCanBus.hpp"

#include <cstdio>

#include <yarp/os/Property.h>

#include <ColorDebug.h>

using namespace roboticslab;

DumpCanBus::DumpCanBus()
    : iCanBus(nullptr),
      iCanBufferFactory(nullptr)
{ }

bool DumpCanBus::configure(yarp::os::ResourceFinder & rf)
{
    CD_DEBUG("%s\n", rf.toString().c_str());

    if (rf.check("help"))
    {
        std::printf("DumpCanBus options:\n");
        std::printf("\t--help (this help)\t--from [file.ini]\t--context [path]\n");
        CD_DEBUG_NO_HEADER("%s\n", rf.toString().c_str());
        return false;
    }

    yarp::os::Property canBusOptions;
    canBusOptions.fromString(rf.toString());
    canBusOptions.put("blockingMode", false); // enforce non-blocking mode
    canBusOptions.put("allowPermissive", false); // always check usage requirements

    if (!canDevice.open(canBusOptions))
    {
        CD_ERROR("CAN device instantiation failed.\n");
        return false;
    }

    canDevice.view(iCanBus);
    canDevice.view(iCanBufferFactory);

    canInputBuffer = iCanBufferFactory->createBuffer(1);

    return yarp::os::Thread::start() || close();
}

bool DumpCanBus::close()
{
    yarp::os::Thread::stop();
    iCanBufferFactory->destroyBuffer(canInputBuffer);
    canDevice.close();
    return true;
}
