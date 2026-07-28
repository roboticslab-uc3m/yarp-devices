// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "CanBusBroker.hpp"

#include <algorithm>
#include <memory>

using namespace roboticslab;
using raw_t = yarp::dev::IPositionControlRaw;

// -----------------------------------------------------------------------------

bool CanBusBroker::positionMove(int j, double ref)
{
    CHECK_JOINT(j);
    return deviceMapper.mapSingleJoint<raw_t, double>(&yarp::dev::IPositionControlRaw::positionMoveRaw, j, ref);
}

// -----------------------------------------------------------------------------

bool CanBusBroker::positionMove(const double * refs)
{
    return deviceMapper.mapAllJoints(&yarp::dev::IPositionControlRaw::positionMoveRaw, refs);
}

// -----------------------------------------------------------------------------

bool CanBusBroker::positionMove(int n_joint, const int * joints, const double * refs)
{
    return deviceMapper.mapJointGroup(&yarp::dev::IPositionControlRaw::positionMoveRaw, n_joint, joints, refs);
}

// -----------------------------------------------------------------------------

bool CanBusBroker::relativeMove(int j, double delta)
{
    CHECK_JOINT(j);
    return deviceMapper.mapSingleJoint<raw_t, double>(&yarp::dev::IPositionControlRaw::relativeMoveRaw, j, delta);
}

// -----------------------------------------------------------------------------

bool CanBusBroker::relativeMove(const double * deltas)
{
    return deviceMapper.mapAllJoints(&yarp::dev::IPositionControlRaw::relativeMoveRaw, deltas);
}

// -----------------------------------------------------------------------------

bool CanBusBroker::relativeMove(int n_joint, const int * joints, const double * deltas)
{
    return deviceMapper.mapJointGroup(&yarp::dev::IPositionControlRaw::relativeMoveRaw, n_joint, joints, deltas);
}

// -----------------------------------------------------------------------------

bool CanBusBroker::checkMotionDone(int j, bool * flag)
{
    CHECK_JOINT(j);
    return deviceMapper.mapSingleJoint<raw_t, bool *>(&yarp::dev::IPositionControlRaw::checkMotionDoneRaw, j, flag);
}

// -----------------------------------------------------------------------------

bool CanBusBroker::checkMotionDone(bool * flag)
{

    auto flags = std::make_unique<bool[]>(deviceMapper.getControlledAxes());

    if (!deviceMapper.mapAllJoints(&yarp::dev::IPositionControlRaw::checkMotionDoneRaw, flags.get()))
    {
        return false;
    }

    *flag = std::all_of(flags.get(), flags.get() + deviceMapper.getControlledAxes(), [](bool b) { return b; });
    return true;
}

// -----------------------------------------------------------------------------

bool CanBusBroker::checkMotionDone(int n_joint, const int * joints, bool * flag)
{

    auto flags = std::make_unique<bool[]>(n_joint);

    if (!deviceMapper.mapJointGroup(&yarp::dev::IPositionControlRaw::checkMotionDoneRaw, n_joint, joints, flags.get()))
    {
        return false;
    }

    *flag = std::all_of(flags.get(), flags.get() + n_joint, [](bool b) { return b; });
    return true;
}

// -----------------------------------------------------------------------------

bool CanBusBroker::setRefSpeed(int j, double spd)
{
    CHECK_JOINT(j);
    return deviceMapper.mapSingleJoint(&yarp::dev::IPositionControlRaw::setRefSpeedRaw, j, spd);
}

// -----------------------------------------------------------------------------

bool CanBusBroker::setRefSpeeds(const double * spds)
{
    return deviceMapper.mapAllJoints(&yarp::dev::IPositionControlRaw::setRefSpeedsRaw, spds);
}

// -----------------------------------------------------------------------------

bool CanBusBroker::setRefSpeeds(int n_joint, const int * joints, const double * spds)
{
    return deviceMapper.mapJointGroup(&yarp::dev::IPositionControlRaw::setRefSpeedsRaw, n_joint, joints, spds);
}

// -----------------------------------------------------------------------------

bool CanBusBroker::setRefAcceleration(int j, double acc)
{
    CHECK_JOINT(j);
    return deviceMapper.mapSingleJoint(&yarp::dev::IPositionControlRaw::setRefAccelerationRaw, j, acc);
}

// -----------------------------------------------------------------------------

bool CanBusBroker::setRefAccelerations(const double * accs)
{
    return deviceMapper.mapAllJoints(&yarp::dev::IPositionControlRaw::setRefAccelerationsRaw, accs);
}

// -----------------------------------------------------------------------------

bool CanBusBroker::setRefAccelerations(int n_joint, const int * joints, const double * accs)
{
    return deviceMapper.mapJointGroup(&yarp::dev::IPositionControlRaw::setRefAccelerationsRaw, n_joint, joints, accs);
}

// -----------------------------------------------------------------------------

bool CanBusBroker::getRefSpeed(int j, double * spd)
{
    CHECK_JOINT(j);
    return deviceMapper.mapSingleJoint(&yarp::dev::IPositionControlRaw::getRefSpeedRaw, j, spd);
}

// -----------------------------------------------------------------------------

bool CanBusBroker::getRefSpeeds(double * spds)
{
    return deviceMapper.mapAllJoints(&yarp::dev::IPositionControlRaw::getRefSpeedsRaw, spds);
}

// -----------------------------------------------------------------------------

bool CanBusBroker::getRefSpeeds(int n_joint, const int * joints, double * spds)
{
    return deviceMapper.mapJointGroup(&yarp::dev::IPositionControlRaw::getRefSpeedsRaw, n_joint, joints, spds);
}

// -----------------------------------------------------------------------------

bool CanBusBroker::getRefAcceleration(int j, double * acc)
{
    CHECK_JOINT(j);
    return deviceMapper.mapSingleJoint(&yarp::dev::IPositionControlRaw::getRefAccelerationRaw, j, acc);
}

// -----------------------------------------------------------------------------

bool CanBusBroker::getRefAccelerations(double * accs)
{
    return deviceMapper.mapAllJoints(&yarp::dev::IPositionControlRaw::getRefAccelerationsRaw, accs);
}

// -----------------------------------------------------------------------------

bool CanBusBroker::getRefAccelerations(int n_joint, const int * joints, double * accs)
{
    return deviceMapper.mapJointGroup(&yarp::dev::IPositionControlRaw::getRefAccelerationsRaw, n_joint, joints, accs);
}

// -----------------------------------------------------------------------------

bool CanBusBroker::stop(int j)
{
    CHECK_JOINT(j);
    return deviceMapper.mapSingleJoint<raw_t>(&yarp::dev::IPositionControlRaw::stopRaw, j);
}

// -----------------------------------------------------------------------------

bool CanBusBroker::stop()
{
    return deviceMapper.mapAllJoints(&yarp::dev::IPositionControlRaw::stopRaw);
}

// -----------------------------------------------------------------------------

bool CanBusBroker::stop(int n_joint, const int * joints)
{
    return deviceMapper.mapJointGroup(&yarp::dev::IPositionControlRaw::stopRaw, n_joint, joints);
}

// -----------------------------------------------------------------------------

bool CanBusBroker::getTargetPosition(int joint, double * ref)
{
    CHECK_JOINT(joint);
    return deviceMapper.mapSingleJoint(&yarp::dev::IPositionControlRaw::getTargetPositionRaw, joint, ref);
}

// -----------------------------------------------------------------------------

bool CanBusBroker::getTargetPositions(double * refs)
{
    return deviceMapper.mapAllJoints(&yarp::dev::IPositionControlRaw::getTargetPositionsRaw, refs);
}

// -----------------------------------------------------------------------------

bool CanBusBroker::getTargetPositions(int n_joint, const int * joints, double * refs)
{
    return deviceMapper.mapJointGroup(&yarp::dev::IPositionControlRaw::getTargetPositionsRaw, n_joint, joints, refs);
}

// -----------------------------------------------------------------------------
