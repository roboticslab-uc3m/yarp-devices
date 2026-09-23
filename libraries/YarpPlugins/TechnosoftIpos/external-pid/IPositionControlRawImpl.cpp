// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "external-pid/TechnosoftIposExternal.hpp"

#include <cmath>

#include <yarp/os/LogStream.h>
#include <yarp/os/SystemClock.h>

#include "CanUtils.hpp"
#include "LogComponent.hpp"

using namespace roboticslab;

// --------------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue TechnosoftIposExternal::positionMoveRaw(int j, double ref)
#else
bool TechnosoftIposExternal::positionMoveRaw(int j, double ref)
#endif
{
    CHECK_JOINT(j);
    CHECK_MODE(VOCAB_CM_POSITION);

    const auto state = limitSwitchState.load();

    if (state == INACTIVE || state == POSITIVE && ref <= max || state == NEGATIVE && ref >= min)
    {
        trajectory.setTargetPosition(yarp::os::SystemClock::nowSystem(),
                                     trajectory.queryPosition(), trajectory.queryVelocity(),
                                     ref, refSpeed, refAcceleration);

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
        return yarp::dev::ReturnValue::return_code::return_value_ok;
#else
        return true;
#endif
    }
    else
    {
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
        return yarp::dev::ReturnValue::return_code::return_value_error_method_failed;
#else
        return false;
#endif
    }
}

// --------------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue TechnosoftIposExternal::relativeMoveRaw(int j, double delta)
#else
bool TechnosoftIposExternal::relativeMoveRaw(int j, double delta)
#endif
{
    CHECK_JOINT(j);
    CHECK_MODE(VOCAB_CM_POSITION);

    const auto state = limitSwitchState.load();

    if (state == INACTIVE || state == POSITIVE && delta <= 0.0 || state == NEGATIVE && delta >= 0.0)
    {
        trajectory.setTargetPosition(yarp::os::SystemClock::nowSystem(),
                                     trajectory.queryPosition(), trajectory.queryVelocity(),
                                     trajectory.queryPosition() + delta, refSpeed, refAcceleration);

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
        return yarp::dev::ReturnValue::return_code::return_value_ok;
#else
        return true;
#endif
    }
    else
    {
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
        return yarp::dev::ReturnValue::return_code::return_value_error_method_failed;
#else
        return false;
#endif
    }
}

// --------------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue TechnosoftIposExternal::checkMotionDoneRaw(int j, bool & flag)
#else
bool TechnosoftIposExternal::checkMotionDoneRaw(int j, bool * flag)
#endif
{
    CHECK_JOINT(j);
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
    flag = !trajectory.isActive();
    return yarp::dev::ReturnValue::return_code::return_value_ok;
#else
    *flag = !trajectory.isActive();
    return true;
#endif
}

// --------------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue TechnosoftIposExternal::setTrajSpeedRaw(int j, double sp)
#else
bool TechnosoftIposExternal::setRefSpeedRaw(int j, double sp)
#endif
{
    CHECK_JOINT(j);

    if (sp < 0.0)
    {
        yCIError(IPOS, id()) << "Illegal reference speed provided:" << sp;
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
        return yarp::dev::ReturnValue::return_code::return_value_error_input_out_of_bounds;
#else
        return false;
#endif
    }
    else if (sp == 0.0)
    {
        yCIWarning(IPOS, id()) << "Reference speed is zero, i.e. no motion will be performed";
    }
    else if (sp > maxVel)
    {
        yCIWarning(IPOS, id()) << "Reference speed exceeds maximum velocity:" << sp << ">" << maxVel.load() << "(will be clipped)";
        sp = maxVel;
    }

    if (trajectory.isActive())
    {
        trajectory.setTargetPosition(yarp::os::SystemClock::nowSystem(),
                                     trajectory.queryPosition(), trajectory.queryVelocity(),
                                     trajectory.getTargetPosition(), sp, refAcceleration);
    }

    refSpeed = sp;
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
    return yarp::dev::ReturnValue::return_code::return_value_ok;
#else
    return true;
#endif
}

// --------------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue TechnosoftIposExternal::setTrajAccelerationRaw(int j, double acc)
#else
bool TechnosoftIposExternal::setRefAccelerationRaw(int j, double acc)
#endif
{
    CHECK_JOINT(j);

    if (acc <= 0.0)
    {
        yCIError(IPOS, id()) << "Illegal reference acceleration provided:" << acc;
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
        return yarp::dev::ReturnValue::return_code::return_value_error_input_out_of_bounds;
#else
        return false;
#endif
    }

    if (trajectory.isActive())
    {
        trajectory.setTargetPosition(yarp::os::SystemClock::nowSystem(),
                                     trajectory.queryPosition(), trajectory.queryVelocity(),
                                     trajectory.getTargetPosition(), refSpeed, acc);
    }

    refAcceleration = acc;
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
    return yarp::dev::ReturnValue::return_code::return_value_ok;
#else
    return true;
#endif
}

// --------------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue TechnosoftIposExternal::getTrajSpeedRaw(int j, double * ref)
#else
bool TechnosoftIposExternal::getRefSpeedRaw(int j, double * ref)
#endif
{
    CHECK_JOINT(j);
    *ref = refSpeed;
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
    return yarp::dev::ReturnValue::return_code::return_value_ok;
#else
    return true;
#endif
}

// --------------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue TechnosoftIposExternal::getTrajAccelerationRaw(int j, double * acc)
#else
bool TechnosoftIposExternal::getRefAccelerationRaw(int j, double * acc)
#endif
{
    CHECK_JOINT(j);
    *acc = refAcceleration;
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
    return yarp::dev::ReturnValue::return_code::return_value_ok;
#else
    return true;
#endif
}

// --------------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue TechnosoftIposExternal::stopRaw(int j)
#else
bool TechnosoftIposExternal::stopRaw(int j)
#endif
{
    CHECK_JOINT(j);

    if (actualControlMode != VOCAB_CM_POSITION && actualControlMode != VOCAB_CM_VELOCITY)
    {
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
        return yarp::dev::ReturnValue::return_code::return_value_error_not_ready;
#else
        return false;
#endif
    }

    if (actualControlMode == VOCAB_CM_VELOCITY && enableCsv)
    {
        commandBuffer.accept(0.0);
    }
    else if (trajectory.isActive())
    {
        trajectory.setTargetVelocity(yarp::os::SystemClock::nowSystem(),
                                     trajectory.queryPosition(), trajectory.queryVelocity(),
                                     0.0, refAcceleration);
    }

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
    return yarp::dev::ReturnValue::return_code::return_value_ok;
#else
    return true;
#endif
}

// --------------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue TechnosoftIposExternal::getTargetPositionRaw(int joint, double * ref)
#else
bool TechnosoftIposExternal::getTargetPositionRaw(int joint, double * ref)
#endif
{
    CHECK_JOINT(joint);
    *ref = trajectory.getTargetPosition();
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
    return yarp::dev::ReturnValue::return_code::return_value_ok;
#else
    return true;
#endif
}

// --------------------------------------------------------------------------------
