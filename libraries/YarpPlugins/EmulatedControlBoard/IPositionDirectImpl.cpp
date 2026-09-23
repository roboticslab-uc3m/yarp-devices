// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "EmulatedControlBoard.hpp"

#include <yarp/os/LogStream.h>

#include "LogComponent.hpp"

// ------------------- IPositionDirect Related --------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue EmulatedControlBoard::setPosition(int j, double ref)
#else
bool EmulatedControlBoard::setPosition(int j, double ref)
#endif
{
    CHECK_JOINT(j);

    if (controlMode != VOCAB_CM_POSITION_DIRECT)
    {
        yCError(ECB) << "will not setPosition() as not in positionDirectMode";
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
        return yarp::dev::ReturnValue::return_code::return_value_error_not_ready;
#else
        return false;
#endif
    }

    targetExposed[j] = ref;
    encRaw[j] = ref * m_encRawExposeds[j];

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
    return yarp::dev::ReturnValue::return_code::return_value_ok;
#else
    return true;
#endif
}

// -----------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue EmulatedControlBoard::setPositions(const double * refs)
#else
bool EmulatedControlBoard::setPositions(const double * refs)
#endif
{
    bool ok = true;

    for (int j = 0; j < m_axes; j++)
    {
        ok &= setPosition(j, refs[j]);
    }

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
    return ok
        ? yarp::dev::ReturnValue::return_code::return_value_ok
        : yarp::dev::ReturnValue::return_code::return_value_error_method_failed;
#else
    return ok;
#endif
}

// -----------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue EmulatedControlBoard::setPositions(int n_joint, const int * joints, const double * refs)
#else
bool EmulatedControlBoard::setPositions(int n_joint, const int * joints, const double * refs)
#endif
{
    bool ok = true;

    for (int i = 0; i < n_joint; i++)
    {
        ok &= setPosition(joints[i], refs[i]);
    }

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
    return ok
        ? yarp::dev::ReturnValue::return_code::return_value_ok
        : yarp::dev::ReturnValue::return_code::return_value_error_method_failed;
#else
    return ok;
#endif
}

// -----------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue EmulatedControlBoard::getRefPosition(int joint, double * ref)
#else
bool EmulatedControlBoard::getRefPosition(int joint, double * ref)
#endif
{
    CHECK_JOINT(joint);

    if (controlMode != VOCAB_CM_POSITION_DIRECT)
    {
        yCError(ECB) << "will not getRefPosition() as not in positionDirectMode";
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
        return yarp::dev::ReturnValue::return_code::return_value_error_not_ready;
#else
        return false;
#endif
    }

    *ref = targetExposed[joint];

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
    return yarp::dev::ReturnValue::return_code::return_value_ok;
#else
    return true;
#endif
}

// -----------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue EmulatedControlBoard::getRefPositions(double * refs)
#else
bool EmulatedControlBoard::getRefPositions(double * refs)
#endif
{
    bool ok = true;

    for (int j = 0; j < m_axes; j++)
    {
        ok &= getRefPosition(j, &refs[j]);
    }

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
    return ok
        ? yarp::dev::ReturnValue::return_code::return_value_ok
        : yarp::dev::ReturnValue::return_code::return_value_error_method_failed;
#else
    return ok;
#endif
}

// -----------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue EmulatedControlBoard::getRefPositions(int n_joint, const int * joints, double * refs)
#else
bool EmulatedControlBoard::getRefPositions(int n_joint, const int * joints, double * refs)
#endif
{
    bool ok = true;

    for (int i = 0; i < n_joint; i++)
    {
        ok &= getRefPosition(joints[i], &refs[i]);
    }

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
    return ok
        ? yarp::dev::ReturnValue::return_code::return_value_ok
        : yarp::dev::ReturnValue::return_code::return_value_error_method_failed;
#else
    return ok;
#endif
}

// -----------------------------------------------------------------------------
