// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "LaunchManipulation.hpp"

namespace teo
{

/************************************************************************/
LaunchManipulation::LaunchManipulation() { }

/************************************************************************/
bool LaunchManipulation::configure(ResourceFinder &rf) {

    //-- /dev/can0 --
    Bottle devCan0 = rf.findGroup("devCan0");
    CD_DEBUG("%s.\n",devCan0.toString().c_str());
    Property optionsDevCan0;
    optionsDevCan0.fromString(devCan0.toString());
    deviceDevCan0.open(optionsDevCan0);
    if (!deviceDevCan0.isValid()) {
        CD_ERROR("deviceDevCan0 instantiation not worked.\n");
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
        return false;
    }

    //-- leftArm --
    Bottle leftArm = rf.findGroup("leftArm");
    CD_DEBUG("%s.\n",leftArm.toString().c_str());
    Property optionsLeftArm;
    optionsLeftArm.fromString(leftArm.toString());
    deviceLeftArm.open(optionsLeftArm);
    if (!deviceLeftArm.isValid()) {
        CD_ERROR("deviceLeftArm instantiation not worked.\n");
        return false;
    }

    //-- rightArm --
    Bottle rightArm = rf.findGroup("rightArm");
    CD_DEBUG("%s.\n",rightArm.toString().c_str());
    Property optionsRightArm;
    optionsRightArm.fromString(rightArm.toString());
    deviceRightArm.open(optionsRightArm);
    if (!deviceRightArm.isValid()) {
        CD_ERROR("deviceRightArm instantiation not worked.\n");
        return false;
    }

    //-- head --
    Bottle head = rf.findGroup("trunk");
    CD_DEBUG("%s.\n",head.toString().c_str());
    Property optionsHead;
    optionsHead.fromString(head.toString());
    deviceHead.open(optionsHead);
    if (!deviceHead.isValid()) {
        CD_ERROR("deviceHead instantiation not worked.\n");
        return false;
    }

    IMultipleWrapper *iwrapperLeftArm, *iwrapperRightArm, *iwrapperHead;

    deviceLeftArm.view(iwrapperLeftArm);
    deviceRightArm.view(iwrapperRightArm);
    deviceHead.view(iwrapperHead);

    PolyDriverList list;
    list.push(&deviceDevCan0, "devCan0");
    list.push(&deviceDevCan1, "devCan1");
    iwrapperLeftArm->attachAll(list);
    iwrapperRightArm->attachAll(list);
    iwrapperHead->attachAll(list);

    return true;
}

/************************************************************************/

bool LaunchManipulation::updateModule() {
    //printf("LaunchManipulation alive...\n");
    return true;
}

/************************************************************************/

bool LaunchManipulation::close() {
    deviceLeftArm.close();
    deviceRightArm.close();
    deviceHead.close();

    deviceDevCan0.close();
    deviceDevCan1.close();
    return true;
}

/************************************************************************/

}  // namespace teo
