// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "LaunchManipulation.hpp"

/************************************************************************/
LaunchManipulation::LaunchManipulation() { }

/************************************************************************/
bool LaunchManipulation::configure(ResourceFinder &rf) {

    //CD_INFO("Using name: %s.\n", rf.find("name").asString().c_str() );

    std::string manipulation_root = ::getenv("MANIPULATION_ROOT");
    CD_INFO("Using root: %s.\n", manipulation_root.c_str() );

    std::string leftArmIni = manipulation_root + "/app/testBodyBot/conf/leftArm.ini";

    Property leftArmOptions;
    if (! leftArmOptions.fromConfigFile(leftArmIni) ) {  //-- Put first because defaults to wiping out.
        CD_ERROR("Could not open %s.\n",leftArmIni.c_str());
        return false;
    }
    CD_SUCCESS("Opened %s.\n",leftArmIni.c_str());
    leftArmOptions.put("name","/teo/leftArm");
    leftArmOptions.put("device","controlboard");
    leftArmOptions.put("subdevice","bodybot");

    leftArmDevice.open(leftArmOptions);
    
    if (!leftArmDevice.isValid()) {
        CD_ERROR("leftArmDevice instantiation not worked.\n");
        CD_ERROR("Be sure CMake \"ENABLE_BodyYarp_bodybot\" variable is set \"ON\"\n");
        CD_ERROR("\"SKIP_bodybot is set\" --> should be --> \"ENABLE_bodybot is set\"\n");
        // robotDevice.close();  // un-needed?
        return false;
    }

    std::string rightArmIni = manipulation_root + "/app/testBodyBot/conf/rightArm.ini";

    Property rightArmOptions;
    if (! rightArmOptions.fromConfigFile(rightArmIni) ) {  //-- Put first because defaults to wiping out.
        CD_ERROR("Could not open %s.\n",rightArmIni.c_str());
        return false;
    }
    CD_SUCCESS("Opened %s.\n",rightArmIni.c_str());
    rightArmOptions.put("name","/teo/rightArm");
    rightArmOptions.put("device","controlboard");
    rightArmOptions.put("subdevice","bodybot");

    rightArmDevice.open(rightArmOptions);

    if (!rightArmDevice.isValid()) {
        CD_ERROR("rightArmDevice instantiation not worked.\n");
        CD_ERROR("Be sure CMake \"ENABLE_BodyYarp_bodybot\" variable is set \"ON\"\n");
        CD_ERROR("\"SKIP_bodybot is set\" --> should be --> \"ENABLE_bodybot is set\"\n");
        // robotDevice.close();  // un-needed?
        return false;
    }

    std::string leftGripperIni = manipulation_root + "/app/testGripperBot/conf/leftGripper.ini";

    Property leftGripperOptions;
    if (! leftGripperOptions.fromConfigFile(leftGripperIni) ) {  //-- Put first because defaults to wiping out.
        CD_ERROR("Could not open %s.\n",leftGripperIni.c_str());
        return false;
    }
    CD_SUCCESS("Opened %s.\n",leftGripperIni.c_str());
    leftGripperOptions.put("name","/teo/leftGripper");
    leftGripperOptions.put("device","controlboard");
    leftGripperOptions.put("subdevice","gripperbot");

    leftGripperDevice.open(leftGripperOptions);

    if (!leftGripperDevice.isValid()) {
        CD_ERROR("leftGripperDevice instantiation not worked.\n");
        CD_ERROR("Be sure CMake \"ENABLE_BodyYarp_gripperbot\" variable is set \"ON\"\n");
        CD_ERROR("\"SKIP_gripperbot is set\" --> should be --> \"ENABLE_gripperbot is set\"\n");
        // robotDevice.close();  // un-needed?
        return false;
    }

    std::string rightGripperIni = manipulation_root + "/app/testGripperBot/conf/rightGripper.ini";

    Property rightGripperOptions;
    if (! rightGripperOptions.fromConfigFile(rightGripperIni) ) {  //-- Put first because defaults to wiping out.
        CD_ERROR("Could not open %s.\n",rightGripperIni.c_str());
        return false;
    }
    CD_SUCCESS("Opened %s.\n",rightGripperIni.c_str());
    rightGripperOptions.put("name","/teo/rightGripper");
    rightGripperOptions.put("device","controlboard");
    rightGripperOptions.put("subdevice","gripperbot");

    rightGripperDevice.open(rightGripperOptions);

    if (!rightGripperDevice.isValid()) {
        CD_ERROR("rightGripperDevice instantiation not worked.\n");
        CD_ERROR("Be sure CMake \"ENABLE_BodyYarp_gripperbot\" variable is set \"ON\"\n");
        CD_ERROR("\"SKIP_gripperbot is set\" --> should be --> \"ENABLE_gripperbot is set\"\n");
        // robotDevice.close();  // un-needed?
        return false;
    }


    return true;
}

/************************************************************************/

bool LaunchManipulation::updateModule() {
    //printf("LaunchManipulation alive...\n");
    return true;
}

/************************************************************************/

bool LaunchManipulation::close() {
    leftArmDevice.close();
    rightArmDevice.close();

    leftGripperDevice.close();
    rightGripperDevice.close();
    return true;
}

/************************************************************************/
