// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "CanBusControlboard.hpp"

#include <ColorDebug.h>

using namespace roboticslab;

// -----------------------------------------------------------------------------

bool CanBusControlboard::setPosition(int j, double ref)
{
    CD_DEBUG("(%d, %f)\n", j, ref);
    CHECK_JOINT(j);
    return deviceMapper.singleJointMapping(j, ref, &yarp::dev::IPositionDirectRaw::setPositionRaw);
}

// -----------------------------------------------------------------------------

bool CanBusControlboard::setPositions(const double * refs)
{
    CD_DEBUG("\n");
    return deviceMapper.fullJointMapping(refs, &yarp::dev::IPositionDirectRaw::setPositionsRaw);
}

// -----------------------------------------------------------------------------

bool CanBusControlboard::setPositions(int n_joint, const int * joints, const double * refs)
{
    CD_DEBUG("\n", n_joint);
    return deviceMapper.multiJointMapping(n_joint, joints, refs, &yarp::dev::IPositionDirectRaw::setPositionsRaw);
}

// -----------------------------------------------------------------------------

bool CanBusControlboard::getRefPosition(int joint, double * ref)
{
    CD_DEBUG("(%d)\n", joint);
    CHECK_JOINT(joint);
    return deviceMapper.singleJointMapping(joint, ref, &yarp::dev::IPositionDirectRaw::getRefPositionRaw);
}

// -----------------------------------------------------------------------------

bool CanBusControlboard::getRefPositions(double * refs)
{
    CD_DEBUG("\n");
    return deviceMapper.fullJointMapping(refs, &yarp::dev::IPositionDirectRaw::getRefPositionsRaw);
}

// -----------------------------------------------------------------------------

bool CanBusControlboard::getRefPositions(int n_joint, const int * joints, double * refs)
{
    CD_DEBUG("(%d)\n", n_joint);
    return deviceMapper.multiJointMapping(n_joint, joints, refs, &yarp::dev::IPositionDirectRaw::getRefPositionsRaw);
}

// -----------------------------------------------------------------------------
