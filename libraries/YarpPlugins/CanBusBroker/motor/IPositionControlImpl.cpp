// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "CanBusBroker.hpp"

#include <algorithm>
#include <memory>

using namespace roboticslab;
using raw_t = yarp::dev::IPositionControlRaw;

// -----------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue CanBusBroker::positionMove(int j, double ref)
#else
bool CanBusBroker::positionMove(int j, double ref)
#endif
{
    CHECK_JOINT(j);
    return deviceMapper.mapSingleJoint<raw_t, double>(&yarp::dev::IPositionControlRaw::positionMoveRaw, j, ref);
}

// -----------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue CanBusBroker::positionMove(const double * refs)
#else
bool CanBusBroker::positionMove(const double * refs)
#endif
{
    return deviceMapper.mapAllJoints(&yarp::dev::IPositionControlRaw::positionMoveRaw, refs);
}

// -----------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue CanBusBroker::positionMove(int n_joint, const int * joints, const double * refs)
#else
bool CanBusBroker::positionMove(int n_joint, const int * joints, const double * refs)
#endif
{
    return deviceMapper.mapJointGroup(&yarp::dev::IPositionControlRaw::positionMoveRaw, n_joint, joints, refs);
}

// -----------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue CanBusBroker::relativeMove(int j, double delta)
#else
bool CanBusBroker::relativeMove(int j, double delta)
#endif
{
    CHECK_JOINT(j);
    return deviceMapper.mapSingleJoint<raw_t, double>(&yarp::dev::IPositionControlRaw::relativeMoveRaw, j, delta);
}

// -----------------------------------------------------------------------------
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue CanBusBroker::relativeMove(const double * deltas)
#else
bool CanBusBroker::relativeMove(const double * deltas)
#endif
{
    return deviceMapper.mapAllJoints(&yarp::dev::IPositionControlRaw::relativeMoveRaw, deltas);
}

// -----------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue CanBusBroker::relativeMove(int n_joint, const int * joints, const double * deltas)
#else
bool CanBusBroker::relativeMove(int n_joint, const int * joints, const double * deltas)
#endif
{
    return deviceMapper.mapJointGroup(&yarp::dev::IPositionControlRaw::relativeMoveRaw, n_joint, joints, deltas);
}

// -----------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue CanBusBroker::checkMotionDone(int j, bool & flag)
#else
bool CanBusBroker::checkMotionDone(int j, bool * flag)
#endif
{
    CHECK_JOINT(j);
    return deviceMapper.mapSingleJoint<raw_t, bool *>(&yarp::dev::IPositionControlRaw::checkMotionDoneRaw, j, flag);
}

// -----------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue CanBusBroker::checkMotionDone(bool & flag)
#else
bool CanBusBroker::checkMotionDone(bool * flag)
#endif
{
    auto flags = std::make_unique<bool[]>(deviceMapper.getControlledAxes());

    if (!deviceMapper.mapAllJoints(&yarp::dev::IPositionControlRaw::checkMotionDoneRaw, flags.get()))
    {
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
        return yarp::dev::ReturnValue::return_code::return_value_error_method_failed;
#else
        return false;
#endif
    }

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
    flag = std::all_of(flags.get(), flags.get() + deviceMapper.getControlledAxes(), [](bool b) { return b; });
    return yarp::dev::ReturnValue::return_code::return_value_ok;
#else
    *flag = std::all_of(flags.get(), flags.get() + deviceMapper.getControlledAxes(), [](bool b) { return b; });
    return true;
#endif
}

// -----------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue CanBusBroker::checkMotionDone(const std::vector<int> & joints, bool & flag)
#else
bool CanBusBroker::checkMotionDone(int n_joint, const int * joints, bool * flag)
#endif
{

    auto flags = std::make_unique<bool[]>(n_joint);

    if (!deviceMapper.mapJointGroup(&yarp::dev::IPositionControlRaw::checkMotionDoneRaw, n_joint, joints, flags.get()))
    {
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
        return yarp::dev::ReturnValue::return_code::return_value_error_method_failed;
#else
        return false;
#endif
    }

    *flag = std::all_of(flags.get(), flags.get() + n_joint, [](bool b) { return b; });
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
    return yarp::dev::ReturnValue::return_code::return_value_ok;
#else
    return true;
#endif
}

// -----------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue CanBusBroker::setTrajSpeed(int j, double spd)
#else
bool CanBusBroker::setRefSpeed(int j, double spd)
#endif
{
    CHECK_JOINT(j);
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
    return deviceMapper.mapSingleJoint(&yarp::dev::IPositionControlRaw::setTrajSpeedRaw, j, spd);
#else
    return deviceMapper.mapSingleJoint(&yarp::dev::IPositionControlRaw::setRefSpeedRaw, j, spd);
#endif
}

// -----------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue CanBusBroker::setTrajSpeeds(const double * spds)
#else
bool CanBusBroker::setRefSpeeds(const double * spds)
#endif
{
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
    return deviceMapper.mapAllJoints(&yarp::dev::IPositionControlRaw::setTrajSpeedsRaw, spds);
#else
    return deviceMapper.mapAllJoints(&yarp::dev::IPositionControlRaw::setRefSpeedsRaw, spds);
#endif
}

// -----------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue CanBusBroker::setTrajSpeeds(int n_joint, const int * joints, const double * spds)
#else
bool CanBusBroker::setRefSpeeds(int n_joint, const int * joints, const double * spds)
#endif
{
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
    return deviceMapper.mapJointGroup(&yarp::dev::IPositionControlRaw::setTrajSpeedsRaw, n_joint, joints, spds);
#else
    return deviceMapper.mapJointGroup(&yarp::dev::IPositionControlRaw::setRefSpeedsRaw, n_joint, joints, spds);
#endif
}

// -----------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue CanBusBroker::setTrajAcceleration(int j, double acc)
#else
bool CanBusBroker::setRefAcceleration(int j, double acc)
#endif
{
    CHECK_JOINT(j);
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
    return deviceMapper.mapSingleJoint(&yarp::dev::IPositionControlRaw::setTrajAccelerationRaw, j, acc);
#else
    return deviceMapper.mapSingleJoint(&yarp::dev::IPositionControlRaw::setRefAccelerationRaw, j, acc);
#endif
}

// -----------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue CanBusBroker::setTrajAccelerations(const double * accs)
#else
bool CanBusBroker::setRefAccelerations(const double * accs)
#endif
{
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
    return deviceMapper.mapAllJoints(&yarp::dev::IPositionControlRaw::setTrajAccelerationsRaw, accs);
#else
    return deviceMapper.mapAllJoints(&yarp::dev::IPositionControlRaw::setRefAccelerationsRaw, accs);
#endif
}

// -----------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue CanBusBroker::setTrajAccelerations(int n_joint, const int * joints, const double * accs)
#else
bool CanBusBroker::setRefAccelerations(int n_joint, const int * joints, const double * accs)
#endif
{
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
    return deviceMapper.mapJointGroup(&yarp::dev::IPositionControlRaw::setTrajAccelerationsRaw, n_joint, joints, accs);
#else
    return deviceMapper.mapJointGroup(&yarp::dev::IPositionControlRaw::setRefAccelerationsRaw, n_joint, joints, accs);
#endif
}

// -----------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue CanBusBroker::getTrajSpeed(int j, double * spd)
#else
bool CanBusBroker::getRefSpeed(int j, double * spd)
#endif
{
    CHECK_JOINT(j);
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
    return deviceMapper.mapSingleJoint(&yarp::dev::IPositionControlRaw::getTrajSpeedRaw, j, spd);
#else
    return deviceMapper.mapSingleJoint(&yarp::dev::IPositionControlRaw::getRefSpeedRaw, j, spd);
#endif
}

// -----------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue CanBusBroker::getTrajSpeeds(double * spds)
#else
bool CanBusBroker::getRefSpeeds(double * spds)
#endif
{
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
    return deviceMapper.mapAllJoints(&yarp::dev::IPositionControlRaw::getTrajSpeedsRaw, spds);
#else
    return deviceMapper.mapAllJoints(&yarp::dev::IPositionControlRaw::getRefSpeedsRaw, spds);
#endif
}

// -----------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue CanBusBroker::getTrajSpeeds(int n_joint, const int * joints, double * spds)
#else
bool CanBusBroker::getRefSpeeds(int n_joint, const int * joints, double * spds)
#endif
{
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
    return deviceMapper.mapJointGroup(&yarp::dev::IPositionControlRaw::getTrajSpeedsRaw, n_joint, joints, spds);
#else
    return deviceMapper.mapJointGroup(&yarp::dev::IPositionControlRaw::getRefSpeedsRaw, n_joint, joints, spds);
#endif
}

// -----------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue CanBusBroker::getTrajAcceleration(int j, double * acc)
#else
bool CanBusBroker::getRefAcceleration(int j, double * acc)
#endif
{
    CHECK_JOINT(j);
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
    return deviceMapper.mapSingleJoint(&yarp::dev::IPositionControlRaw::getTrajAccelerationRaw, j, acc);
#else
    return deviceMapper.mapSingleJoint(&yarp::dev::IPositionControlRaw::getRefAccelerationRaw, j, acc);
#endif
}

// -----------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue CanBusBroker::getTrajAccelerations(double * accs)
#else
bool CanBusBroker::getRefAccelerations(double * accs)
#endif
{
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
    return deviceMapper.mapAllJoints(&yarp::dev::IPositionControlRaw::getTrajAccelerationsRaw, accs);
#else
    return deviceMapper.mapAllJoints(&yarp::dev::IPositionControlRaw::getRefAccelerationsRaw, accs);
#endif
}

// -----------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue CanBusBroker::getTrajAccelerations(int n_joint, const int * joints, double * accs)
#else
bool CanBusBroker::getRefAccelerations(int n_joint, const int * joints, double * accs)
#endif
{
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
    return deviceMapper.mapJointGroup(&yarp::dev::IPositionControlRaw::getTrajAccelerationsRaw, n_joint, joints, accs);
#else
    return deviceMapper.mapJointGroup(&yarp::dev::IPositionControlRaw::getRefAccelerationsRaw, n_joint, joints, accs);
#endif
}

// -----------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue CanBusBroker::stop(int j)
#else
bool CanBusBroker::stop(int j)
#endif
{
    CHECK_JOINT(j);
    return deviceMapper.mapSingleJoint<raw_t>(&yarp::dev::IPositionControlRaw::stopRaw, j);
}

// -----------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue CanBusBroker::stop()
#else
bool CanBusBroker::stop()
#endif
{
    return deviceMapper.mapAllJoints(&yarp::dev::IPositionControlRaw::stopRaw);
}

// -----------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue CanBusBroker::stop(int n_joint, const int * joints)
#else
bool CanBusBroker::stop(int n_joint, const int * joints)
#endif
{
    return deviceMapper.mapJointGroup(&yarp::dev::IPositionControlRaw::stopRaw, n_joint, joints);
}

// -----------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue CanBusBroker::getTargetPosition(int joint, double * ref)
#else
bool CanBusBroker::getTargetPosition(int joint, double * ref)
#endif
{
    CHECK_JOINT(joint);
    return deviceMapper.mapSingleJoint(&yarp::dev::IPositionControlRaw::getTargetPositionRaw, joint, ref);
}

// -----------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue CanBusBroker::getTargetPositions(double * refs)
#else
bool CanBusBroker::getTargetPositions(double * refs)
#endif
{
    return deviceMapper.mapAllJoints(&yarp::dev::IPositionControlRaw::getTargetPositionsRaw, refs);
}

// -----------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue CanBusBroker::getTargetPositions(int n_joint, const int * joints, double * refs)
#else
bool CanBusBroker::getTargetPositions(int n_joint, const int * joints, double * refs)
#endif
{
    return deviceMapper.mapJointGroup(&yarp::dev::IPositionControlRaw::getTargetPositionsRaw, n_joint, joints, refs);
}

// -----------------------------------------------------------------------------
