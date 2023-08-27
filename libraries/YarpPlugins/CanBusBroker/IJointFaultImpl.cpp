// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "CanBusBroker.hpp"

#include <yarp/os/Log.h>

#include "LogComponent.hpp"

using namespace roboticslab;

// -----------------------------------------------------------------------------

bool CanBusBroker::getLastJointFault(int axis, int & fault, std::string & message)
{
    yCTrace(CBB, "%d", axis);
    CHECK_JOINT(axis);
    auto fn = &yarp::dev::IJointFaultRaw::getLastJointFaultRaw;
    return deviceMapper.mapSingleJoint<yarp::dev::IJointFaultRaw, int &, std::string &>(fn, axis, fault, message);
}

// -----------------------------------------------------------------------------
