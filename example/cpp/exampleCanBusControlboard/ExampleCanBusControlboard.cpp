// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "ExampleCanBusControlboard.hpp"

/************************************************************************/
teo::ExampleCanBusControlboard::ExampleCanBusControlboard() { }

/************************************************************************/
bool teo::ExampleCanBusControlboard::configure(ResourceFinder &rf) {

    Property options;
    options.fromString(rf.toString());  //-- Allow options like stream_state=0.
    options.put("device","controlboardwrapper2");
    options.put("subdevice","bodybot");

    robotDevice.open(options);
    
    if (!robotDevice.isValid()) {
        CD_ERROR("Class instantiation not worked.\n");
        CD_ERROR("Be sure CMake \"ENABLE_BodyYarp_bodybot\" variable is set \"ON\"\n");
        CD_ERROR("\"SKIP_bodybot is set\" --> should be --> \"ENABLE_bodybot is set\"\n");
        // robotDevice.close();  // un-needed?
        return false;
    }

    return true;
}

/************************************************************************/
bool teo::ExampleCanBusControlboard::updateModule() {
    //printf("ExampleCanBusControlboard alive...\n");
    return true;
}

