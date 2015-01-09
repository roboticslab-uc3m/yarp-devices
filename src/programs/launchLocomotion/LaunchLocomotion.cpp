// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "LaunchLocomotion.hpp"

/************************************************************************/
LaunchLocomotion::LaunchLocomotion() { }

/************************************************************************/
bool LaunchLocomotion::configure(ResourceFinder &rf) {

    //CD_INFO("Using name: %s.\n", rf.find("name").asString().c_str() );

    std::string manipulation_root = ::getenv("MANIPULATION_ROOT");
    CD_INFO("Using root: %s.\n", manipulation_root.c_str() );

    std::string leftLegIni = manipulation_root + "/app/testBodyBot/conf/leftLeg.ini";

    Property leftLegOptions;
    if (! leftLegOptions.fromConfigFile(leftLegIni) ) {  //-- Put first because defaults to wiping out.
        CD_ERROR("Could not open %s.\n",leftLegIni.c_str());
        return false;
    }
    CD_SUCCESS("Opened %s.\n",leftLegIni.c_str());
    leftLegOptions.put("name","/teo/leftLeg");
    leftLegOptions.put("device","controlboard");
    leftLegOptions.put("subdevice","bodybot");

    leftLegDevice.open(leftLegOptions);
    
    if (!leftLegDevice.isValid()) {
        CD_ERROR("leftLegDevice instantiation not worked.\n");
        CD_ERROR("Be sure CMake \"ENABLE_BodyYarp_bodybot\" variable is set \"ON\"\n");
        CD_ERROR("\"SKIP_bodybot is set\" --> should be --> \"ENABLE_bodybot is set\"\n");
        // robotDevice.close();  // un-needed?
        return false;
    }

    std::string rightLegIni = manipulation_root + "/app/testBodyBot/conf/rightLeg.ini";

    Property rightLegOptions;
    if (! rightLegOptions.fromConfigFile(rightLegIni) ) {  //-- Put first because defaults to wiping out.
        CD_ERROR("Could not open %s.\n",rightLegIni.c_str());
        return false;
    }
    CD_SUCCESS("Opened %s.\n",rightLegIni.c_str());
    rightLegOptions.put("name","/teo/rightLeg");
    rightLegOptions.put("device","controlboard");
    rightLegOptions.put("subdevice","bodybot");

    rightLegDevice.open(rightLegOptions);

    if (!rightLegDevice.isValid()) {
        CD_ERROR("rightLegDevice instantiation not worked.\n");
        CD_ERROR("Be sure CMake \"ENABLE_BodyYarp_bodybot\" variable is set \"ON\"\n");
        CD_ERROR("\"SKIP_bodybot is set\" --> should be --> \"ENABLE_bodybot is set\"\n");
        // robotDevice.close();  // un-needed?
        return false;
    }


    return true;
}

/************************************************************************/

bool LaunchLocomotion::updateModule() {
    //printf("LaunchLocomotion alive...\n");
    return true;
}

/************************************************************************/

bool LaunchLocomotion::close() {
    leftLegDevice.close();
    rightLegDevice.close();

    return true;
}

/************************************************************************/
