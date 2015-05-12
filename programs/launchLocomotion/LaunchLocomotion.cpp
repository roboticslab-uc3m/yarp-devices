// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "LaunchLocomotion.hpp"

namespace teo
{

/************************************************************************/
LaunchLocomotion::LaunchLocomotion() { }

/************************************************************************/
bool LaunchLocomotion::configure(ResourceFinder &rf) {

    //-- /dev/can0 --
    Bottle devCan0 = rf.findGroup("devCan0");
    CD_DEBUG("%s.\n",devCan0.toString().c_str());
    Property optionsDevCan0;
    optionsDevCan0.fromString(devCan0.toString());
    deviceDevCan0.open(optionsDevCan0);
    
    if (!deviceDevCan0.isValid()) {
        CD_ERROR("deviceDevCan0 instantiation not worked.\n");
        // robotDevice.close();  // un-needed?
        return false;
    }

    //-- /dev/can1 --
    Bottle devCan1 = rf.findGroup("devCan1");
    CD_DEBUG("%s.\n",devCan1.toString().c_str());
    Property optionsDevCan1;
    optionsDevCan1.fromString(devCan1.toString());
    deviceDevCan1.open(optionsDevCan1);

    if (!deviceDevCan1.isValid()) {
        CD_ERROR("deviceDevCan1 instantiation not worked.\n");
        // robotDevice.close();  // un-needed?
        return false;
    }

    //-- leftLeg --
    Bottle leftLeg = rf.findGroup("leftLeg");
    CD_DEBUG("%s.\n",leftLeg.toString().c_str());
    Property optionsLeftLeg;
    optionsLeftLeg.fromString(leftLeg.toString());
    deviceLeftLeg.open(optionsLeftLeg);

    //-- rightLeg --
    Bottle rightLeg = rf.findGroup("rightLeg");
    CD_DEBUG("%s.\n",rightLeg.toString().c_str());
    Property optionsRightLeg;
    optionsRightLeg.fromString(rightLeg.toString());
    deviceRightLeg.open(optionsRightLeg);

    //-- trunk --
    Bottle trunk = rf.findGroup("trunk");
    CD_DEBUG("%s.\n",trunk.toString().c_str());
    Property optionsTrunk;
    optionsTrunk.fromString(trunk.toString());
    deviceTrunk.open(optionsTrunk);

    IMultipleWrapper *iwrapperLeftLeg, *iwrapperRightLeg, *iwrapperTrunk;

    deviceLeftLeg.view(iwrapperLeftLeg);
    deviceRightLeg.view(iwrapperRightLeg);
    deviceTrunk.view(iwrapperTrunk);

    PolyDriverList list;
    list.push(&deviceDevCan0, "devCan0");
    list.push(&deviceDevCan1, "devCan1");
    iwrapperLeftLeg->attachAll(list);
    iwrapperRightLeg->attachAll(list);
    iwrapperTrunk->attachAll(list);

    return true;
}

/************************************************************************/

bool LaunchLocomotion::updateModule() {
    //printf("LaunchLocomotion alive...\n");
    return true;
}

/************************************************************************/

bool LaunchLocomotion::close() {
    deviceDevCan0.close();
    deviceDevCan1.close();
    return true;
}

/************************************************************************/

}  // namespace teo
