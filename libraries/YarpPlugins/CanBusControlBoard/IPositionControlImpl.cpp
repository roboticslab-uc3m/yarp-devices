// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "CanBusControlBoard.hpp"

#include <algorithm>
#include <memory>

#include <yarp/os/Log.h>

#include "LogComponent.hpp"

using namespace roboticslab;
using raw_t = yarp::dev::IPositionControlRaw;

// -----------------------------------------------------------------------------

bool CanBusControlBoard::positionMove(int j, double ref)
{
    yCTrace(CBCB, "%d %f", j, ref);
    CHECK_JOINT(j);
    return deviceMapper.mapSingleJoint<raw_t, double>(&yarp::dev::IPositionControlRaw::positionMoveRaw, j, ref);
}

// -----------------------------------------------------------------------------

bool CanBusControlBoard::positionMove(const double * refs)
{
    yCTrace(CBCB, "");
    return deviceMapper.mapAllJoints(&yarp::dev::IPositionControlRaw::positionMoveRaw, refs);
}

// -----------------------------------------------------------------------------

bool CanBusControlBoard::positionMove(int n_joint, const int * joints, const double * refs)
{
    yCTrace(CBCB, "%d", n_joint);
    return deviceMapper.mapJointGroup(&yarp::dev::IPositionControlRaw::positionMoveRaw, n_joint, joints, refs);
}

// -----------------------------------------------------------------------------

bool CanBusControlBoard::relativeMove(int j, double delta)
{
    yCTrace(CBCB, "%d %f", j, delta);
    CHECK_JOINT(j);
    return deviceMapper.mapSingleJoint<raw_t, double>(&yarp::dev::IPositionControlRaw::relativeMoveRaw, j, delta);
}

// -----------------------------------------------------------------------------

bool CanBusControlBoard::relativeMove(const double * deltas)
{
    yCTrace(CBCB, "");
    return deviceMapper.mapAllJoints(&yarp::dev::IPositionControlRaw::relativeMoveRaw, deltas);
}

// -----------------------------------------------------------------------------

bool CanBusControlBoard::relativeMove(int n_joint, const int * joints, const double * deltas)
{
    yCTrace(CBCB, "%d", n_joint);
    return deviceMapper.mapJointGroup(&yarp::dev::IPositionControlRaw::relativeMoveRaw, n_joint, joints, deltas);
}

// -----------------------------------------------------------------------------

bool CanBusControlBoard::checkMotionDone(int j, bool * flag)
{
    yCTrace(CBCB, "%d", j);
    CHECK_JOINT(j);
    return deviceMapper.mapSingleJoint<raw_t, bool *>(&yarp::dev::IPositionControlRaw::checkMotionDoneRaw, j, flag);
}

// -----------------------------------------------------------------------------

bool CanBusControlBoard::checkMotionDone(bool * flag)
{
    yCTrace(CBCB, "");

    auto flags = std::make_unique<bool[]>(deviceMapper.getControlledAxes());

    if (!deviceMapper.mapAllJoints(&yarp::dev::IPositionControlRaw::checkMotionDoneRaw, flags.get()))
    {
        return false;
    }

    *flag = std::all_of(flags.get(), flags.get() + deviceMapper.getControlledAxes(), [](bool b) { return b; });
    return true;
}

// -----------------------------------------------------------------------------

bool CanBusControlBoard::checkMotionDone(int n_joint, const int * joints, bool * flag)
{
    yCTrace(CBCB, "%d", n_joint);

    auto flags = std::make_unique<bool[]>(n_joint);

    if (!deviceMapper.mapJointGroup(&yarp::dev::IPositionControlRaw::checkMotionDoneRaw, n_joint, joints, flags.get()))
    {
        return false;
    }

    *flag = std::all_of(flags.get(), flags.get() + n_joint, [](bool b) { return b; });
    return true;
}

// -----------------------------------------------------------------------------

bool CanBusControlBoard::setRefSpeed(int j, double spd)
{
    yCTrace(CBCB, "%d %f", j, spd);
    CHECK_JOINT(j);
    return deviceMapper.mapSingleJoint(&yarp::dev::IPositionControlRaw::setRefSpeedRaw, j, spd);
}

// -----------------------------------------------------------------------------

bool CanBusControlBoard::setRefSpeeds(const double * spds)
{
    yCTrace(CBCB, "");
    return deviceMapper.mapAllJoints(&yarp::dev::IPositionControlRaw::setRefSpeedsRaw, spds);
}

// -----------------------------------------------------------------------------

bool CanBusControlBoard::setRefSpeeds(int n_joint, const int * joints, const double * spds)
{
    yCTrace(CBCB, "%d", n_joint);
    return deviceMapper.mapJointGroup(&yarp::dev::IPositionControlRaw::setRefSpeedsRaw, n_joint, joints, spds);
}

// -----------------------------------------------------------------------------

bool CanBusControlBoard::setRefAcceleration(int j, double acc)
{
    yCTrace(CBCB, "%d %f", j, acc);
    CHECK_JOINT(j);
    return deviceMapper.mapSingleJoint(&yarp::dev::IPositionControlRaw::setRefAccelerationRaw, j, acc);
}

// -----------------------------------------------------------------------------

bool CanBusControlBoard::setRefAccelerations(const double * accs)
{
    yCTrace(CBCB, "");
    return deviceMapper.mapAllJoints(&yarp::dev::IPositionControlRaw::setRefAccelerationsRaw, accs);
}

// -----------------------------------------------------------------------------

bool CanBusControlBoard::setRefAccelerations(int n_joint, const int * joints, const double * accs)
{
    yCTrace(CBCB, "%d", n_joint);
    return deviceMapper.mapJointGroup(&yarp::dev::IPositionControlRaw::setRefAccelerationsRaw, n_joint, joints, accs);
}

// -----------------------------------------------------------------------------

bool CanBusControlBoard::getRefSpeed(int j, double * spd)
{
    yCTrace(CBCB, "%d", j);
    CHECK_JOINT(j);
    return deviceMapper.mapSingleJoint(&yarp::dev::IPositionControlRaw::getRefSpeedRaw, j, spd);
}

// -----------------------------------------------------------------------------

bool CanBusControlBoard::getRefSpeeds(double * spds)
{
    yCTrace(CBCB, "");
    return deviceMapper.mapAllJoints(&yarp::dev::IPositionControlRaw::getRefSpeedsRaw, spds);
}

// -----------------------------------------------------------------------------

bool CanBusControlBoard::getRefSpeeds(int n_joint, const int * joints, double * spds)
{
    yCTrace(CBCB, "%d", n_joint);
    return deviceMapper.mapJointGroup(&yarp::dev::IPositionControlRaw::getRefSpeedsRaw, n_joint, joints, spds);
}

// -----------------------------------------------------------------------------

bool CanBusControlBoard::getRefAcceleration(int j, double * acc)
{
    yCTrace(CBCB, "%d", j);
    CHECK_JOINT(j);
    return deviceMapper.mapSingleJoint(&yarp::dev::IPositionControlRaw::getRefAccelerationRaw, j, acc);
}

// -----------------------------------------------------------------------------

bool CanBusControlBoard::getRefAccelerations(double * accs)
{
    yCTrace(CBCB, "");
    return deviceMapper.mapAllJoints(&yarp::dev::IPositionControlRaw::getRefAccelerationsRaw, accs);
}

// -----------------------------------------------------------------------------

bool CanBusControlBoard::getRefAccelerations(int n_joint, const int * joints, double * accs)
{
    yCTrace(CBCB, "%d", n_joint);
    return deviceMapper.mapJointGroup(&yarp::dev::IPositionControlRaw::getRefAccelerationsRaw, n_joint, joints, accs);
}

// -----------------------------------------------------------------------------

bool CanBusControlBoard::stop(int j)
{
    yCTrace(CBCB, "%d", j);
    CHECK_JOINT(j);
    return deviceMapper.mapSingleJoint<raw_t>(&yarp::dev::IPositionControlRaw::stopRaw, j);
}

// -----------------------------------------------------------------------------

bool CanBusControlBoard::stop()
{
    yCTrace(CBCB, "");
    return deviceMapper.mapAllJoints(&yarp::dev::IPositionControlRaw::stopRaw);
}

// -----------------------------------------------------------------------------

bool CanBusControlBoard::stop(int n_joint, const int * joints)
{
    yCTrace(CBCB, "%d", n_joint);
    return deviceMapper.mapJointGroup(&yarp::dev::IPositionControlRaw::stopRaw, n_joint, joints);
}

// -----------------------------------------------------------------------------

bool CanBusControlBoard::getTargetPosition(int joint, double * ref)
{
    yCTrace(CBCB, "%d", joint);
    CHECK_JOINT(joint);
    return deviceMapper.mapSingleJoint(&yarp::dev::IPositionControlRaw::getTargetPositionRaw, joint, ref);
}

// -----------------------------------------------------------------------------

bool CanBusControlBoard::getTargetPositions(double * refs)
{
    yCTrace(CBCB, "");
    return deviceMapper.mapAllJoints(&yarp::dev::IPositionControlRaw::getTargetPositionsRaw, refs);
}

// -----------------------------------------------------------------------------

bool CanBusControlBoard::getTargetPositions(int n_joint, const int * joints, double * refs)
{
    yCTrace(CBCB, "%d", n_joint);
    return deviceMapper.mapJointGroup(&yarp::dev::IPositionControlRaw::getTargetPositionsRaw, n_joint, joints, refs);
}

// -----------------------------------------------------------------------------
