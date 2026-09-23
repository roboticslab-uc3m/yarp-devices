// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "CanBusBroker.hpp"

using namespace roboticslab;

// -----------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue CanBusBroker::setPosLimits(int axis, double min, double max)
#else
bool CanBusBroker::setLimits(int axis, double min, double max)
#endif
{
    CHECK_JOINT(axis);
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
    return deviceMapper.mapSingleJoint(&yarp::dev::IControlLimitsRaw::setPosLimitsRaw, axis, min, max);
#else
    return deviceMapper.mapSingleJoint(&yarp::dev::IControlLimitsRaw::setLimitsRaw, axis, min, max);
#endif
}

// -----------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue CanBusBroker::getPosLimits(int axis, double * min, double * max)
#else
bool CanBusBroker::getLimits(int axis, double * min, double * max)
#endif
{
    CHECK_JOINT(axis);
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
    return deviceMapper.mapSingleJoint(&yarp::dev::IControlLimitsRaw::getPosLimitsRaw, axis, min, max);
#else
    return deviceMapper.mapSingleJoint(&yarp::dev::IControlLimitsRaw::getLimitsRaw, axis, min, max);
#endif
}

// -----------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue CanBusBroker::setVelLimits(int axis, double min, double max)
#else
bool CanBusBroker::setVelLimits(int axis, double min, double max)
#endif
{
    CHECK_JOINT(axis);
    return deviceMapper.mapSingleJoint(&yarp::dev::IControlLimitsRaw::setVelLimitsRaw, axis, min, max);
}

// -----------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue CanBusBroker::getVelLimits(int axis, double * min, double * max)
#else
bool CanBusBroker::getVelLimits(int axis, double * min, double * max)
#endif
{
    CHECK_JOINT(axis);
    return deviceMapper.mapSingleJoint(&yarp::dev::IControlLimitsRaw::getVelLimitsRaw, axis, min, max);
}

// -----------------------------------------------------------------------------
