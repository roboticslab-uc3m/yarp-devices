// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "DextraCanControlboard.hpp"

using namespace roboticslab;

bool DextraCanControlboard::setCanBusPtr(yarp::dev::ICanBus * canDevicePtr)
{
    synapse->configure(canDevicePtr);
    return true;
}

bool DextraCanControlboard::interpretMessage(const yarp::dev::CanMessage & message)
{
    return true;
}
