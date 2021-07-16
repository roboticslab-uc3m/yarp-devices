// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "CanBusControlboard.hpp"

#include <yarp/os/Log.h>

#include "LogComponent.hpp"

using namespace roboticslab;

// -----------------------------------------------------------------------------

bool CanBusControlboard::getAxisName(int axis, std::string & name)
{
    yCTrace(CBCB, "%d", axis);
    CHECK_JOINT(axis);
    auto fn = &yarp::dev::IAxisInfoRaw::getAxisNameRaw;
    return deviceMapper.mapSingleJoint<yarp::dev::IAxisInfoRaw, std::string &>(fn, axis, name);
}

// -----------------------------------------------------------------------------

bool CanBusControlboard::getJointType(int axis, yarp::dev::JointTypeEnum & type)
{
    yCTrace(CBCB, "%d", axis);
    CHECK_JOINT(axis);
    auto fn = &yarp::dev::IAxisInfoRaw::getJointTypeRaw;
    return deviceMapper.mapSingleJoint<yarp::dev::IAxisInfoRaw, yarp::dev::JointTypeEnum &>(fn, axis, type);
}

// -----------------------------------------------------------------------------
