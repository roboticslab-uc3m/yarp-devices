// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "CanBusControlboard.hpp"

#include <ColorDebug.h>

using namespace roboticslab;

// -----------------------------------------------------------------------------

bool CanBusControlboard::getAxisName(int axis, std::string & name)
{
    CD_DEBUG("(%d)\n", axis);
    CHECK_JOINT(axis);
    auto fn = &yarp::dev::IAxisInfoRaw::getAxisNameRaw;
    return deviceMapper.singleJointMapping<yarp::dev::IAxisInfoRaw, std::string &>(axis, name, fn);
}

// -----------------------------------------------------------------------------

bool CanBusControlboard::getJointType(int axis, yarp::dev::JointTypeEnum & type)
{
    CD_DEBUG("(%d)\n", axis);
    CHECK_JOINT(axis);
    auto fn = &yarp::dev::IAxisInfoRaw::getJointTypeRaw;
    return deviceMapper.singleJointMapping<yarp::dev::IAxisInfoRaw, yarp::dev::JointTypeEnum &>(axis, type, fn);
}

// -----------------------------------------------------------------------------
