// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "DumpCanBus.hpp"

namespace teo
{

/************************************************************************/
DumpCanBus::DumpCanBus() { }

/************************************************************************/
bool DumpCanBus::configure(ResourceFinder &rf) {

    if(rf.check("help")) {
        printf("DumpCanBus options:\n");
        printf("\t--help (this help)\t--from [file.ini]\t--context [path]\n");
        CD_DEBUG_NO_HEADER("%s\n",rf.toString().c_str());
        return false;
    }

    //-- /dev/can0 --
    Bottle devCan0 = rf.findGroup("devCan0");
    CD_DEBUG("%s\n",devCan0.toString().c_str());
    Property optionsDevCan0;
    optionsDevCan0.fromString(devCan0.toString());
    deviceDevCan0.open(optionsDevCan0);
    if (!deviceDevCan0.isValid()) {
        CD_ERROR("deviceDevCan0 instantiation not worked.\n");
        return false;
    }

    return true;
}

/************************************************************************/

bool DumpCanBus::updateModule() {
    //printf("DumpCanBus alive...\n");
    return true;
}

/************************************************************************/

bool DumpCanBus::close() {

    deviceDevCan0.close();

    return true;
}

/************************************************************************/

}  // namespace teo
