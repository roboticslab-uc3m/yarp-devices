// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "OneCanBusOneWrapper.hpp"

namespace teo
{

/************************************************************************/
OneCanBusOneWrapper::OneCanBusOneWrapper() { }

/************************************************************************/
bool OneCanBusOneWrapper::configure(yarp::os::ResourceFinder &rf) {

    if(rf.check("help")) {
        printf("OneCanBusOneWrapper options:\n");
        printf("\t--help (this help)\t--from [file.ini]\t--context [path]\n");
        CD_DEBUG_NO_HEADER("%s\n",rf.toString().c_str());
        return false;
    }

    //-- Variable that stores the mode
    CD_DEBUG("mode activated: %s\n", rf.find("mode").asString().c_str());
    std::string mode = rf.check("mode",yarp::os::Value("position"),"position/velocity mode").asString();


    //-- /dev/can0 --
    yarp::os::Bottle devCan0 = rf.findGroup("devCan0");
    CD_DEBUG("%s\n",devCan0.toString().c_str());
    yarp::os::Property optionsDevCan0;
    optionsDevCan0.fromString(devCan0.toString());
    deviceDevCan0.open(optionsDevCan0);
    if (!deviceDevCan0.isValid()) {
        CD_ERROR("deviceDevCan0 instantiation not worked.\n");
        return false;
    }

    //-- wrapper0 --
    yarp::os::Bottle wrapper0 = rf.findGroup("wrapper0");
    CD_DEBUG("%s\n",wrapper0.toString().c_str());
    yarp::os::Property optionsWrapper0;
    optionsWrapper0.fromString(wrapper0.toString());
    deviceWrapper0.open(optionsWrapper0);
    if (!deviceWrapper0.isValid()) {
        CD_ERROR("deviceWrapper0 instantiation not worked.\n");
        return false;
    }

    yarp::dev::IMultipleWrapper *iWrapper0;

    deviceWrapper0.view(iWrapper0);

    yarp::dev::PolyDriverList list;
    list.push(&deviceDevCan0, "devCan0");
    iWrapper0->attachAll(list);

    return true;
}

/************************************************************************/

bool OneCanBusOneWrapper::updateModule() {
    //printf("OneCanBusOneWrapper alive...\n");
    return true;
}

/************************************************************************/

bool OneCanBusOneWrapper::close() {
    deviceWrapper0.close();

    deviceDevCan0.close();

    return true;
}

/************************************************************************/

}  // namespace teo
