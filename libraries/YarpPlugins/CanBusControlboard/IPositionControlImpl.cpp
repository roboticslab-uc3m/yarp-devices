// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "CanBusControlboard.hpp"

#include <algorithm>
#include <memory>

#include <yarp/os/Log.h>

using namespace roboticslab;
using raw_t = yarp::dev::IPositionControlRaw;

// -----------------------------------------------------------------------------

bool CanBusControlboard::positionMove(int j, double ref)
{
    yTrace("%d %f", j, ref);
    CHECK_JOINT(j);
    return deviceMapper.mapSingleJoint<raw_t, double>(&yarp::dev::IPositionControlRaw::positionMoveRaw, j, ref);
}

// -----------------------------------------------------------------------------

bool CanBusControlboard::positionMove(const double * refs)
{
    yTrace("");
    return deviceMapper.mapAllJoints(&yarp::dev::IPositionControlRaw::positionMoveRaw, refs);
}

// -----------------------------------------------------------------------------

bool CanBusControlboard::positionMove(int n_joint, const int * joints, const double * refs)
{
    yTrace("%d", n_joint);
    return deviceMapper.mapJointGroup(&yarp::dev::IPositionControlRaw::positionMoveRaw, n_joint, joints, refs);
}

// -----------------------------------------------------------------------------

bool CanBusControlboard::relativeMove(int j, double delta)
{
    yTrace("%d %f", j, delta);
    CHECK_JOINT(j);
    return deviceMapper.mapSingleJoint<raw_t, double>(&yarp::dev::IPositionControlRaw::relativeMoveRaw, j, delta);
}

// -----------------------------------------------------------------------------

bool CanBusControlboard::relativeMove(const double * deltas)
{
    yTrace("");
    return deviceMapper.mapAllJoints(&yarp::dev::IPositionControlRaw::relativeMoveRaw, deltas);
}

// -----------------------------------------------------------------------------

bool CanBusControlboard::relativeMove(int n_joint, const int * joints, const double * deltas)
{
    yTrace("%d", n_joint);
    return deviceMapper.mapJointGroup(&yarp::dev::IPositionControlRaw::relativeMoveRaw, n_joint, joints, deltas);
}

// -----------------------------------------------------------------------------

bool CanBusControlboard::checkMotionDone(int j, bool * flag)
{
    yTrace("%d", j);
    CHECK_JOINT(j);
    return deviceMapper.mapSingleJoint<raw_t, bool *>(&yarp::dev::IPositionControlRaw::checkMotionDoneRaw, j, flag);
}

// -----------------------------------------------------------------------------

bool CanBusControlboard::checkMotionDone(bool * flag)
{
    yTrace("");

    auto flags = std::make_unique<bool[]>(deviceMapper.getControlledAxes());

    if (!deviceMapper.mapAllJoints(&yarp::dev::IPositionControlRaw::checkMotionDoneRaw, flags.get()))
    {
        return false;
    }

    *flag = std::all_of(flags.get(), flags.get() + deviceMapper.getControlledAxes(), [](bool b) { return b; });
    return true;
}

// -----------------------------------------------------------------------------

bool CanBusControlboard::checkMotionDone(int n_joint, const int * joints, bool * flag)
{
    yTrace("%d", n_joint);

    auto flags = std::make_unique<bool[]>(n_joint);

    if (!deviceMapper.mapJointGroup(&yarp::dev::IPositionControlRaw::checkMotionDoneRaw, n_joint, joints, flags.get()))
    {
        return false;
    }

    *flag = std::all_of(flags.get(), flags.get() + n_joint, [](bool b) { return b; });
    return true;
}

// -----------------------------------------------------------------------------

bool CanBusControlboard::setRefSpeed(int j, double spd)
{
    yTrace("%d %f", j, spd);
    CHECK_JOINT(j);
    return deviceMapper.mapSingleJoint(&yarp::dev::IPositionControlRaw::setRefSpeedRaw, j, spd);
}

// -----------------------------------------------------------------------------

bool CanBusControlboard::setRefSpeeds(const double * spds)
{
    yTrace("");
    return deviceMapper.mapAllJoints(&yarp::dev::IPositionControlRaw::setRefSpeedsRaw, spds);
}

// -----------------------------------------------------------------------------

bool CanBusControlboard::setRefSpeeds(int n_joint, const int * joints, const double * spds)
{
    yTrace("%d", n_joint);
    return deviceMapper.mapJointGroup(&yarp::dev::IPositionControlRaw::setRefSpeedsRaw, n_joint, joints, spds);
}

// -----------------------------------------------------------------------------

bool CanBusControlboard::setRefAcceleration(int j, double acc)
{
    yTrace("%d %f", j, acc);
    CHECK_JOINT(j);
    return deviceMapper.mapSingleJoint(&yarp::dev::IPositionControlRaw::setRefAccelerationRaw, j, acc);
}

// -----------------------------------------------------------------------------

bool CanBusControlboard::setRefAccelerations(const double * accs)
{
    yTrace("");
    return deviceMapper.mapAllJoints(&yarp::dev::IPositionControlRaw::setRefAccelerationsRaw, accs);
}

// -----------------------------------------------------------------------------

bool CanBusControlboard::setRefAccelerations(int n_joint, const int * joints, const double * accs)
{
    yTrace("%d", n_joint);
    return deviceMapper.mapJointGroup(&yarp::dev::IPositionControlRaw::setRefAccelerationsRaw, n_joint, joints, accs);
}

// -----------------------------------------------------------------------------

bool CanBusControlboard::getRefSpeed(int j, double * spd)
{
    yTrace("%d", j);
    CHECK_JOINT(j);
    return deviceMapper.mapSingleJoint(&yarp::dev::IPositionControlRaw::getRefSpeedRaw, j, spd);
}

// -----------------------------------------------------------------------------

bool CanBusControlboard::getRefSpeeds(double * spds)
{
    yTrace("");
    return deviceMapper.mapAllJoints(&yarp::dev::IPositionControlRaw::getRefSpeedsRaw, spds);
}

// -----------------------------------------------------------------------------

bool CanBusControlboard::getRefSpeeds(int n_joint, const int * joints, double * spds)
{
    yTrace("%d", n_joint);
    return deviceMapper.mapJointGroup(&yarp::dev::IPositionControlRaw::getRefSpeedsRaw, n_joint, joints, spds);
}

// -----------------------------------------------------------------------------

bool CanBusControlboard::getRefAcceleration(int j, double * acc)
{
    yTrace("%d", j);
    CHECK_JOINT(j);
    return deviceMapper.mapSingleJoint(&yarp::dev::IPositionControlRaw::getRefAccelerationRaw, j, acc);
}

// -----------------------------------------------------------------------------

bool CanBusControlboard::getRefAccelerations(double * accs)
{
    yTrace("");
    return deviceMapper.mapAllJoints(&yarp::dev::IPositionControlRaw::getRefAccelerationsRaw, accs);
}

// -----------------------------------------------------------------------------

bool CanBusControlboard::getRefAccelerations(int n_joint, const int * joints, double * accs)
{
    yTrace("%d", n_joint);
    return deviceMapper.mapJointGroup(&yarp::dev::IPositionControlRaw::getRefAccelerationsRaw, n_joint, joints, accs);
}

// -----------------------------------------------------------------------------

bool CanBusControlboard::stop(int j)
{
    yTrace("%d", j);
    CHECK_JOINT(j);
    return deviceMapper.mapSingleJoint<raw_t>(&yarp::dev::IPositionControlRaw::stopRaw, j);
}

// -----------------------------------------------------------------------------

bool CanBusControlboard::stop()
{
    yTrace("");
    return deviceMapper.mapAllJoints(&yarp::dev::IPositionControlRaw::stopRaw);
}

// -----------------------------------------------------------------------------

bool CanBusControlboard::stop(int n_joint, const int * joints)
{
    yTrace("%d", n_joint);
    return deviceMapper.mapJointGroup(&yarp::dev::IPositionControlRaw::stopRaw, n_joint, joints);
}

// -----------------------------------------------------------------------------

bool CanBusControlboard::getTargetPosition(int joint, double * ref)
{
    yTrace("%d", joint);
    CHECK_JOINT(joint);
    return deviceMapper.mapSingleJoint(&yarp::dev::IPositionControlRaw::getTargetPositionRaw, joint, ref);
}

// -----------------------------------------------------------------------------

bool CanBusControlboard::getTargetPositions(double * refs)
{
    yTrace("");
    return deviceMapper.mapAllJoints(&yarp::dev::IPositionControlRaw::getTargetPositionsRaw, refs);
}

// -----------------------------------------------------------------------------

bool CanBusControlboard::getTargetPositions(int n_joint, const int * joints, double * refs)
{
    yTrace("%d", n_joint);
    return deviceMapper.mapJointGroup(&yarp::dev::IPositionControlRaw::getTargetPositionsRaw, n_joint, joints, refs);
}

// -----------------------------------------------------------------------------
