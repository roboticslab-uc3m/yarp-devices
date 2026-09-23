// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "CanBusBroker.hpp"

using namespace roboticslab;

// -----------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue CanBusBroker::getImpedance(int j, double * stiffness, double * damping)
#else
bool CanBusBroker::getImpedance(int j, double * stiffness, double * damping)
#endif
{
    CHECK_JOINT(j);
    return deviceMapper.mapSingleJoint(&yarp::dev::IImpedanceControlRaw::getImpedanceRaw, j, stiffness, damping);
}

// -----------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue CanBusBroker::setImpedance(int j, double stiffness, double damping)
#else
bool CanBusBroker::setImpedance(int j, double stiffness, double damping)
#endif
{
    CHECK_JOINT(j);
    return deviceMapper.mapSingleJoint(&yarp::dev::IImpedanceControlRaw::setImpedanceRaw, j, stiffness, damping);
}

// -----------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue CanBusBroker::setImpedanceOffset(int j, double offset)
#else
bool CanBusBroker::setImpedanceOffset(int j, double offset)
#endif
{
    CHECK_JOINT(j);
    return deviceMapper.mapSingleJoint(&yarp::dev::IImpedanceControlRaw::setImpedanceOffsetRaw, j, offset);
}

// -----------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue CanBusBroker::getImpedanceOffset(int j, double * offset)
#else
bool CanBusBroker::getImpedanceOffset(int j, double * offset)
#endif
{
    CHECK_JOINT(j);
    return deviceMapper.mapSingleJoint(&yarp::dev::IImpedanceControlRaw::getImpedanceOffsetRaw, j, offset);
}

// -----------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue CanBusBroker::getCurrentImpedanceLimit(int j, double * min_stiff, double * max_stiff, double * min_damp, double * max_damp)
#else
bool CanBusBroker::getCurrentImpedanceLimit(int j, double * min_stiff, double * max_stiff, double * min_damp, double * max_damp)
#endif
{
    CHECK_JOINT(j);
    return deviceMapper.mapSingleJoint(&yarp::dev::IImpedanceControlRaw::getCurrentImpedanceLimitRaw, j, min_stiff, max_stiff, min_damp, max_damp);
}

// -----------------------------------------------------------------------------
