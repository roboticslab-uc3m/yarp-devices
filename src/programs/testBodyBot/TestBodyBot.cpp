// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "TestBodyBot.hpp"

/************************************************************************/
teo::TestBodyBot::TestBodyBot() { }

/************************************************************************/
bool teo::TestBodyBot::configure(ResourceFinder &rf) {

    Property options;
    options.fromString(rf.toString());  //-- Allow options like stream_state=0.
    options.put("device","controlboard");
    options.put("subdevice","bodybot");

    robotDevice.open(options);
    
    if (!robotDevice.isValid()) {
        CD_ERROR("Class instantiation not worked.\n");
        CD_ERROR("Be sure CMake \"ENABLE_BodyYarp_gripperbot\" variable is set \"ON\"\n");
        CD_ERROR("\"SKIP_gripperbot is set\" --> should be --> \"ENABLE_gripperbot is set\"\n");
        // robotDevice.close();  // un-needed?
        return false;
    }

    return true;
}

/************************************************************************/
bool teo::TestBodyBot::updateModule() {
    //printf("TestBodyBot alive...\n");
    return true;
}

