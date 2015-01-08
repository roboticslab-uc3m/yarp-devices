// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "TestBodyBot.hpp"

/************************************************************************/
TestBodyBot::TestBodyBot() { }

/************************************************************************/
bool TestBodyBot::configure(ResourceFinder &rf) {

    std::string robot_name = rf.check("robot",Value(DEFAULT_ROBOT_NAME),"Robot name.").asString();

    CD_INFO("Using robot: %s.\n",robot_name.c_str());
    
    std::string gripperName(robot_name.c_str());
    gripperName += "/gripper";
    CD_INFO("Using gripperName: %s.\n",gripperName.c_str());

    Property options;
    options.fromString(rf.toString());  //-- Allow options like stream_state=0.
    options.put("device","controlboard");
    options.put("subdevice","gripperbot");
    options.put("name",gripperName);

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
bool TestBodyBot::updateModule() {
    //printf("TestBodyBot alive...\n");
    return true;
}

