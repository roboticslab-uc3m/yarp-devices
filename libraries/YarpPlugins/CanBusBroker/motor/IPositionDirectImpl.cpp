// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "CanBusBroker.hpp"

#include <yarp/os/Log.h>

#include "LogComponent.hpp"

using namespace roboticslab;

// -----------------------------------------------------------------------------

bool CanBusBroker::setPosition(int j, double ref)
{
    yCTrace(CBB, "%d %f", j, ref);
    CHECK_JOINT(j);
    return deviceMapper.mapSingleJoint(&yarp::dev::IPositionDirectRaw::setPositionRaw, j, ref);
}

// -----------------------------------------------------------------------------

bool CanBusBroker::setPositions(const double * refs)
{
    yCTrace(CBB, "");
    return deviceMapper.mapAllJoints(&yarp::dev::IPositionDirectRaw::setPositionsRaw, refs);
}

// -----------------------------------------------------------------------------

bool CanBusBroker::setPositions(int n_joint, const int * joints, const double * refs)
{
    yCTrace(CBB, "%d", n_joint);
    return deviceMapper.mapJointGroup(&yarp::dev::IPositionDirectRaw::setPositionsRaw, n_joint, joints, refs);
}

// -----------------------------------------------------------------------------

bool CanBusBroker::getRefPosition(int joint, double * ref)
{
    yCTrace(CBB, "%d", joint);
    CHECK_JOINT(joint);
    return deviceMapper.mapSingleJoint(&yarp::dev::IPositionDirectRaw::getRefPositionRaw, joint, ref);
}

// -----------------------------------------------------------------------------

bool CanBusBroker::getRefPositions(double * refs)
{
    yCTrace(CBB, "");
    return deviceMapper.mapAllJoints(&yarp::dev::IPositionDirectRaw::getRefPositionsRaw, refs);
}

// -----------------------------------------------------------------------------

bool CanBusBroker::getRefPositions(int n_joint, const int * joints, double * refs)
{
    yCTrace(CBB, "%d", n_joint);
    return deviceMapper.mapJointGroup(&yarp::dev::IPositionDirectRaw::getRefPositionsRaw, n_joint, joints, refs);
}

// -----------------------------------------------------------------------------
