// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "EmulatedControlBoard.hpp"

#include <yarp/os/LogStream.h>

#include "LogComponent.hpp"

// ------------------- IControlLimits Related ------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue EmulatedControlBoard::setPosLimits(int axis, double min, double max)
#else
bool EmulatedControlBoard::setLimits(int axis, double min, double max)
#endif
{
    CHECK_JOINT(axis);

    m_minLimits[axis] = min;
    m_maxLimits[axis] = max;

    yCDebug(ECB, "Range of axis %d set to: %f to %f", axis, min, max);

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
    return yarp::dev::ReturnValue_ok;
#else
    return true;
#endif
}

// -----------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue EmulatedControlBoard::getPosLimits(int axis, double * min, double * max)
#else
bool EmulatedControlBoard::getLimits(int axis, double * min, double * max)
#endif
{
    CHECK_JOINT(axis);

    *min = m_minLimits[axis];
    *max = m_maxLimits[axis];

    yCDebug(ECB, "Range of axis %d read: %f to %f", axis, *min, *max);

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
    return yarp::dev::ReturnValue_ok;
#else
    return true;
#endif
}

// -----------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue EmulatedControlBoard::setVelLimits(int axis, double min, double max)
#else
bool EmulatedControlBoard::setVelLimits(int axis, double min, double max)
#endif
{
    yCWarning(ECB, "setVelLimits() not implemented");
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
    return yarp::dev::ReturnValue_error_not_implemented_by_device;
#else
    return false;
#endif
}

// -----------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue EmulatedControlBoard::getVelLimits(int axis, double * min, double * max)
#else
bool EmulatedControlBoard::getVelLimits(int axis, double * min, double * max)
#endif
{
    CHECK_JOINT(axis);

    // yarpmotorgui's defaults (partitem.cpp)
    *min = -100.0;
    *max = 100.0;

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
    return yarp::dev::ReturnValue_ok;
#else
    return true;
#endif
}

// -----------------------------------------------------------------------------
