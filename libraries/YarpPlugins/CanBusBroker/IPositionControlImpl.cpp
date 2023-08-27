// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "CanBusBroker.hpp"

#include <algorithm>
#include <memory>

#include <yarp/os/Log.h>

#include "LogComponent.hpp"

using namespace roboticslab;
using raw_t = yarp::dev::IPositionControlRaw;

// -----------------------------------------------------------------------------

bool CanBusBroker::positionMove(int j, double ref)
{
    yCTrace(CBB, "%d %f", j, ref);
    CHECK_JOINT(j);
    return deviceMapper.mapSingleJoint<raw_t, double>(&yarp::dev::IPositionControlRaw::positionMoveRaw, j, ref);
}

// -----------------------------------------------------------------------------

bool CanBusBroker::positionMove(const double * refs)
{
    yCTrace(CBB, "");
    return deviceMapper.mapAllJoints(&yarp::dev::IPositionControlRaw::positionMoveRaw, refs);
}

// -----------------------------------------------------------------------------

bool CanBusBroker::positionMove(int n_joint, const int * joints, const double * refs)
{
    yCTrace(CBB, "%d", n_joint);
    return deviceMapper.mapJointGroup(&yarp::dev::IPositionControlRaw::positionMoveRaw, n_joint, joints, refs);
}

// -----------------------------------------------------------------------------

bool CanBusBroker::relativeMove(int j, double delta)
{
    yCTrace(CBB, "%d %f", j, delta);
    CHECK_JOINT(j);
    return deviceMapper.mapSingleJoint<raw_t, double>(&yarp::dev::IPositionControlRaw::relativeMoveRaw, j, delta);
}

// -----------------------------------------------------------------------------

bool CanBusBroker::relativeMove(const double * deltas)
{
    yCTrace(CBB, "");
    return deviceMapper.mapAllJoints(&yarp::dev::IPositionControlRaw::relativeMoveRaw, deltas);
}

// -----------------------------------------------------------------------------

bool CanBusBroker::relativeMove(int n_joint, const int * joints, const double * deltas)
{
    yCTrace(CBB, "%d", n_joint);
    return deviceMapper.mapJointGroup(&yarp::dev::IPositionControlRaw::relativeMoveRaw, n_joint, joints, deltas);
}

// -----------------------------------------------------------------------------

bool CanBusBroker::checkMotionDone(int j, bool * flag)
{
    yCTrace(CBB, "%d", j);
    CHECK_JOINT(j);
    return deviceMapper.mapSingleJoint<raw_t, bool *>(&yarp::dev::IPositionControlRaw::checkMotionDoneRaw, j, flag);
}

// -----------------------------------------------------------------------------

bool CanBusBroker::checkMotionDone(bool * flag)
{
    yCTrace(CBB, "");

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
    yCTrace(CBB, "%d", n_joint);

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
    yCTrace(CBB, "%d %f", j, spd);
    CHECK_JOINT(j);
    return deviceMapper.mapSingleJoint(&yarp::dev::IPositionControlRaw::setRefSpeedRaw, j, spd);
}

// -----------------------------------------------------------------------------

bool CanBusBroker::setRefSpeeds(const double * spds)
{
    yCTrace(CBB, "");
    return deviceMapper.mapAllJoints(&yarp::dev::IPositionControlRaw::setRefSpeedsRaw, spds);
}

// -----------------------------------------------------------------------------

bool CanBusBroker::setRefSpeeds(int n_joint, const int * joints, const double * spds)
{
    yCTrace(CBB, "%d", n_joint);
    return deviceMapper.mapJointGroup(&yarp::dev::IPositionControlRaw::setRefSpeedsRaw, n_joint, joints, spds);
}

// -----------------------------------------------------------------------------

bool CanBusBroker::setRefAcceleration(int j, double acc)
{
    yCTrace(CBB, "%d %f", j, acc);
    CHECK_JOINT(j);
    return deviceMapper.mapSingleJoint(&yarp::dev::IPositionControlRaw::setRefAccelerationRaw, j, acc);
}

// -----------------------------------------------------------------------------

bool CanBusBroker::setRefAccelerations(const double * accs)
{
    yCTrace(CBB, "");
    return deviceMapper.mapAllJoints(&yarp::dev::IPositionControlRaw::setRefAccelerationsRaw, accs);
}

// -----------------------------------------------------------------------------

bool CanBusBroker::setRefAccelerations(int n_joint, const int * joints, const double * accs)
{
    yCTrace(CBB, "%d", n_joint);
    return deviceMapper.mapJointGroup(&yarp::dev::IPositionControlRaw::setRefAccelerationsRaw, n_joint, joints, accs);
}

// -----------------------------------------------------------------------------

bool CanBusBroker::getRefSpeed(int j, double * spd)
{
    yCTrace(CBB, "%d", j);
    CHECK_JOINT(j);
    return deviceMapper.mapSingleJoint(&yarp::dev::IPositionControlRaw::getRefSpeedRaw, j, spd);
}

// -----------------------------------------------------------------------------

bool CanBusBroker::getRefSpeeds(double * spds)
{
    yCTrace(CBB, "");
    return deviceMapper.mapAllJoints(&yarp::dev::IPositionControlRaw::getRefSpeedsRaw, spds);
}

// -----------------------------------------------------------------------------

bool CanBusBroker::getRefSpeeds(int n_joint, const int * joints, double * spds)
{
    yCTrace(CBB, "%d", n_joint);
    return deviceMapper.mapJointGroup(&yarp::dev::IPositionControlRaw::getRefSpeedsRaw, n_joint, joints, spds);
}

// -----------------------------------------------------------------------------

bool CanBusBroker::getRefAcceleration(int j, double * acc)
{
    yCTrace(CBB, "%d", j);
    CHECK_JOINT(j);
    return deviceMapper.mapSingleJoint(&yarp::dev::IPositionControlRaw::getRefAccelerationRaw, j, acc);
}

// -----------------------------------------------------------------------------

bool CanBusBroker::getRefAccelerations(double * accs)
{
    yCTrace(CBB, "");
    return deviceMapper.mapAllJoints(&yarp::dev::IPositionControlRaw::getRefAccelerationsRaw, accs);
}

// -----------------------------------------------------------------------------

bool CanBusBroker::getRefAccelerations(int n_joint, const int * joints, double * accs)
{
    yCTrace(CBB, "%d", n_joint);
    return deviceMapper.mapJointGroup(&yarp::dev::IPositionControlRaw::getRefAccelerationsRaw, n_joint, joints, accs);
}

// -----------------------------------------------------------------------------

bool CanBusBroker::stop(int j)
{
    yCTrace(CBB, "%d", j);
    CHECK_JOINT(j);
    return deviceMapper.mapSingleJoint<raw_t>(&yarp::dev::IPositionControlRaw::stopRaw, j);
}

// -----------------------------------------------------------------------------

bool CanBusBroker::stop()
{
    yCTrace(CBB, "");
    return deviceMapper.mapAllJoints(&yarp::dev::IPositionControlRaw::stopRaw);
}

// -----------------------------------------------------------------------------

bool CanBusBroker::stop(int n_joint, const int * joints)
{
    yCTrace(CBB, "%d", n_joint);
    return deviceMapper.mapJointGroup(&yarp::dev::IPositionControlRaw::stopRaw, n_joint, joints);
}

// -----------------------------------------------------------------------------

bool CanBusBroker::getTargetPosition(int joint, double * ref)
{
    yCTrace(CBB, "%d", joint);
    CHECK_JOINT(joint);
    return deviceMapper.mapSingleJoint(&yarp::dev::IPositionControlRaw::getTargetPositionRaw, joint, ref);
}

// -----------------------------------------------------------------------------

bool CanBusBroker::getTargetPositions(double * refs)
{
    yCTrace(CBB, "");
    return deviceMapper.mapAllJoints(&yarp::dev::IPositionControlRaw::getTargetPositionsRaw, refs);
}

// -----------------------------------------------------------------------------

bool CanBusBroker::getTargetPositions(int n_joint, const int * joints, double * refs)
{
    yCTrace(CBB, "%d", n_joint);
    return deviceMapper.mapJointGroup(&yarp::dev::IPositionControlRaw::getTargetPositionsRaw, n_joint, joints, refs);
}

// -----------------------------------------------------------------------------
