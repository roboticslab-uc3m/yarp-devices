// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "CanBusControlboard.hpp"

#include <ColorDebug.h>

using namespace roboticslab;

// -----------------------------------------------------------------------------

bool CanBusControlboard::getAxes(int *axes)
{
    CD_DEBUG("\n");
    *axes = deviceMapper.getControlledAxes();
    return true;
}

// -----------------------------------------------------------------------------

bool CanBusControlboard::positionMove(int j, double ref)
{
    CD_DEBUG("(%d, %f)\n", j, ref);
    CHECK_JOINT(j);
    return deviceMapper.singleJointMapping(j, ref, &yarp::dev::IPositionControlRaw::positionMoveRaw);
}

// -----------------------------------------------------------------------------

bool CanBusControlboard::positionMove(const double * refs)
{
    CD_DEBUG("\n");
    return deviceMapper.fullJointMapping(refs, &yarp::dev::IPositionControlRaw::positionMoveRaw);
}

// -----------------------------------------------------------------------------

bool CanBusControlboard::positionMove(int n_joint, const int * joints, const double * refs)
{
    CD_DEBUG("\n");
    return deviceMapper.multiJointMapping(n_joint, joints, refs, &yarp::dev::IPositionControlRaw::positionMoveRaw);
}

// -----------------------------------------------------------------------------

bool CanBusControlboard::relativeMove(int j, double delta)
{
    CD_DEBUG("(%d, %f)\n", j, delta);
    CHECK_JOINT(j);
    return deviceMapper.singleJointMapping(j, delta, &yarp::dev::IPositionControlRaw::relativeMoveRaw);
}

// -----------------------------------------------------------------------------

bool CanBusControlboard::relativeMove(const double * deltas)
{
    CD_DEBUG("\n");
    return deviceMapper.fullJointMapping(deltas, &yarp::dev::IPositionControlRaw::relativeMoveRaw);
}

// -----------------------------------------------------------------------------

bool CanBusControlboard::relativeMove(int n_joint, const int * joints, const double * deltas)
{
    CD_DEBUG("\n");
    return deviceMapper.multiJointMapping(n_joint, joints, deltas, &yarp::dev::IPositionControlRaw::relativeMoveRaw);
}

// -----------------------------------------------------------------------------

bool CanBusControlboard::checkMotionDone(int j, bool * flag)
{
    CD_DEBUG("(%d)\n", j);
    CHECK_JOINT(j);
    return deviceMapper.singleJointMapping(j, flag, &yarp::dev::IPositionControlRaw::checkMotionDoneRaw);
}

// -----------------------------------------------------------------------------

bool CanBusControlboard::checkMotionDone(bool * flags)
{
    CD_DEBUG("\n");
    return deviceMapper.fullJointMapping(flags, &yarp::dev::IPositionControlRaw::checkMotionDoneRaw);
}

// -----------------------------------------------------------------------------

bool CanBusControlboard::checkMotionDone(int n_joint, const int * joints, bool * flags)
{
    CD_DEBUG("\n");
    return deviceMapper.multiJointMapping(n_joint, joints, flags, &yarp::dev::IPositionControlRaw::checkMotionDoneRaw);
}

// -----------------------------------------------------------------------------

bool CanBusControlboard::setRefSpeed(int j, double spd)
{
    CD_DEBUG("(%d, %f)\n", j, spd);
    CHECK_JOINT(j);
    return deviceMapper.singleJointMapping(j, spd, &yarp::dev::IPositionControlRaw::setRefSpeedRaw);
}

// -----------------------------------------------------------------------------

bool CanBusControlboard::setRefSpeeds(const double * spds)
{
    CD_DEBUG("\n");
    return deviceMapper.fullJointMapping(spds, &yarp::dev::IPositionControlRaw::setRefSpeedsRaw);
}

// -----------------------------------------------------------------------------

bool CanBusControlboard::setRefSpeeds(int n_joint, const int * joints, const double * spds)
{
    CD_DEBUG("\n");
    return deviceMapper.multiJointMapping(n_joint, joints, spds, &yarp::dev::IPositionControlRaw::setRefSpeedsRaw);
}

// -----------------------------------------------------------------------------

bool CanBusControlboard::setRefAcceleration(int j, double acc)
{
    CD_DEBUG("(%d, %f)\n", j, acc);
    CHECK_JOINT(j);
    return deviceMapper.singleJointMapping(j, acc, &yarp::dev::IPositionControlRaw::setRefAccelerationRaw);
}

// -----------------------------------------------------------------------------

bool CanBusControlboard::setRefAccelerations(const double * accs)
{
    CD_DEBUG("\n");
    return deviceMapper.fullJointMapping(accs, &yarp::dev::IPositionControlRaw::setRefAccelerationsRaw);
}

// -----------------------------------------------------------------------------

bool CanBusControlboard::setRefAccelerations(int n_joint, const int * joints, const double * accs)
{
    CD_DEBUG("\n");
    return deviceMapper.multiJointMapping(n_joint, joints, accs, &yarp::dev::IPositionControlRaw::setRefAccelerationsRaw);
}

// -----------------------------------------------------------------------------

bool CanBusControlboard::getRefSpeed(int j, double * spd)
{
    CD_DEBUG("(%d)\n", j);
    CHECK_JOINT(j);
    return deviceMapper.singleJointMapping(j, spd, &yarp::dev::IPositionControlRaw::getRefSpeedRaw);
}

// -----------------------------------------------------------------------------

bool CanBusControlboard::getRefSpeeds(double * spds)
{
    CD_DEBUG("\n");
    return deviceMapper.fullJointMapping(spds, &yarp::dev::IPositionControlRaw::getRefSpeedsRaw);
}

// -----------------------------------------------------------------------------

bool CanBusControlboard::getRefSpeeds(int n_joint, const int * joints, double * spds)
{
    CD_DEBUG("\n");
    return deviceMapper.multiJointMapping(n_joint, joints, spds, &yarp::dev::IPositionControlRaw::getRefSpeedsRaw);
}

// -----------------------------------------------------------------------------

bool CanBusControlboard::getRefAcceleration(int j, double * acc)
{
    CD_DEBUG("(%d)\n", j);
    CHECK_JOINT(j);
    return deviceMapper.singleJointMapping(j, acc, &yarp::dev::IPositionControlRaw::getRefAccelerationRaw);
}

// -----------------------------------------------------------------------------

bool CanBusControlboard::getRefAccelerations(double * accs)
{
    CD_DEBUG("\n");
    return deviceMapper.fullJointMapping(accs, &yarp::dev::IPositionControlRaw::getRefAccelerationsRaw);
}

// -----------------------------------------------------------------------------

bool CanBusControlboard::getRefAccelerations(int n_joint, const int * joints, double * accs)
{
    CD_DEBUG("\n");
    return deviceMapper.multiJointMapping(n_joint, joints, accs, &yarp::dev::IPositionControlRaw::getRefAccelerationsRaw);
}

// -----------------------------------------------------------------------------

bool CanBusControlboard::stop(int j)
{
    CD_DEBUG("(%d)\n", j);
    CHECK_JOINT(j);

    int localAxis;
    yarp::dev::IPositionControlRaw * p = deviceMapper.getDevice(j, &localAxis).iPositionControlRaw;

    return p ? p->stopRaw(localAxis) : false;
}

// -----------------------------------------------------------------------------

bool CanBusControlboard::stop()
{
    CD_DEBUG("\n");

    const std::vector<RawDevice> & rawDevices = deviceMapper.getDevices();
    bool ok = true;

    for (int i = 0; i < rawDevices.size(); i++)
    {
        yarp::dev::IPositionControlRaw * p = rawDevices[i].iPositionControlRaw;
        ok &= p ? p->stopRaw() : false;
    }

    return ok;
}

// -----------------------------------------------------------------------------

bool CanBusControlboard::stop(int n_joint, const int * joints)
{
    CD_DEBUG("\n");
    bool ok = true;

    for (const auto & t : deviceMapper.getDevices(n_joint, joints))
    {
        yarp::dev::IPositionControlRaw * p = std::get<0>(t)->iPositionControlRaw;
        const auto & localIndices = deviceMapper.computeLocalIndices(std::get<1>(t), joints, std::get<2>(t));
        ok &= p ? p->stopRaw(std::get<1>(t), localIndices.data()) : false;
    }

    return ok;
}

// -----------------------------------------------------------------------------

bool CanBusControlboard::getTargetPosition(int joint, double * ref)
{
    CD_DEBUG("(%d)\n", joint);
    CHECK_JOINT(joint);
    return deviceMapper.singleJointMapping(joint, ref, &yarp::dev::IPositionControlRaw::getTargetPositionRaw);

}

// -----------------------------------------------------------------------------

bool CanBusControlboard::getTargetPositions(double * refs)
{
    CD_DEBUG("\n");
    return deviceMapper.fullJointMapping(refs, &yarp::dev::IPositionControlRaw::getTargetPositionsRaw);
}

// -----------------------------------------------------------------------------

bool CanBusControlboard::getTargetPositions(int n_joint, const int * joints, double * refs)
{
    CD_DEBUG("\n");
    return deviceMapper.multiJointMapping(n_joint, joints, refs, &yarp::dev::IPositionControlRaw::getTargetPositionsRaw);
}

// -----------------------------------------------------------------------------
