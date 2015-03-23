// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "LaunchLocomotion.hpp"

namespace teo
{

/************************************************************************/
LaunchLocomotion::LaunchLocomotion() { }

/************************************************************************/
bool LaunchLocomotion::configure(ResourceFinder &rf) {

    //-- left leg --
    std::string leftLegIni = rf.findFileByName("../locomotion/leftLeg.ini");

    Property leftLegOptions;
    if (! leftLegOptions.fromConfigFile(leftLegIni) ) {  //-- Put first because defaults to wiping out.
        CD_ERROR("Could not configure from \"leftLeg.ini\".\n");
        return false;
    }
    CD_SUCCESS("Configured left leg from %s.\n",leftLegIni.c_str());
    leftLegOptions.put("name","/teo/leftLeg");
    leftLegOptions.put("device","controlboardwrapper2");
    leftLegOptions.put("subdevice","bodybot");
    if (rf.check("home")) leftLegOptions.put("home",1);
    if (rf.check("reset")) leftLegOptions.put("reset",1);

    leftLegDevice.open(leftLegOptions);
    
    if (!leftLegDevice.isValid()) {
        CD_ERROR("leftLegDevice instantiation not worked.\n");
        CD_ERROR("Be sure CMake \"ENABLE_BodyYarp_bodybot\" variable is set \"ON\"\n");
        CD_ERROR("\"SKIP_bodybot is set\" --> should be --> \"ENABLE_bodybot is set\"\n");
        // robotDevice.close();  // un-needed?
        return false;
    }

    //-- right leg --
    std::string rightLegIni = rf.findFileByName("../locomotion/rightLeg.ini");

    Property rightLegOptions;
    if (! rightLegOptions.fromConfigFile(rightLegIni) ) {  //-- Put first because defaults to wiping out.
        CD_ERROR("Could not configure from \"rightLeg.ini\".\n");
        return false;
    }
    CD_SUCCESS("Configured right leg from %s.\n",rightLegIni.c_str());
    rightLegOptions.put("name","/teo/rightLeg");
    rightLegOptions.put("device","controlboardwrapper2");
    rightLegOptions.put("subdevice","bodybot");
    if (rf.check("home")) rightLegOptions.put("home",1);
    if (rf.check("reset")) rightLegOptions.put("reset",1);

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

}  // namespace teo
