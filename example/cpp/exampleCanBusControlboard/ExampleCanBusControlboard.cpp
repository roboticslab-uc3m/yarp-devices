// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "ExampleCanBusControlboard.hpp"

/************************************************************************/
roboticslab::ExampleCanBusControlboard::ExampleCanBusControlboard() { }

/************************************************************************/
bool roboticslab::ExampleCanBusControlboard::configure(yarp::os::ResourceFinder &rf)
{
    yarp::os::Property options;
    options.fromString(rf.toString());  //-- Allow options like stream_state=0.
    options.put("device","controlboardwrapper2");
    options.put("subdevice","CanBusControlboard");

    robotDevice.open(options);

    if (!robotDevice.isValid())
    {
        CD_ERROR("Class instantiation not worked.\n");
        CD_ERROR("Be sure CMake \"ENABLE_YarpPlugins_CanBusControlboard\" variable is set \"ON\"\n");
        CD_ERROR("\"SKIP_CanBusControlboard is set\" --> should be --> \"ENABLE_CanBusControlboard is set\"\n");
        // robotDevice.close();  // un-needed?
        return false;
    }

    return true;
}

/************************************************************************/
bool roboticslab::ExampleCanBusControlboard::updateModule()
{
    //printf("ExampleCanBusControlboard alive...\n");
    return true;
}
