// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "TechnosoftIposBase.hpp"

using namespace roboticslab;

// --------------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue TechnosoftIposBase::getAxes(std::size_t & ax)
{
    ax = 1;
    return yarp::dev::ReturnValue_ok;
}
#else
bool TechnosoftIposBase::getAxes(int * ax)
{
    *ax = 1;
    return true;
}
#endif

// -----------------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue TechnosoftIposBase::resetEncoderRaw(int j)
#else
bool TechnosoftIposBase::resetEncoderRaw(int j)
#endif
{
    CHECK_JOINT(j);
    return setEncoderRaw(j, 0.0);
}

// -----------------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue TechnosoftIposBase::setEncoderRaw(int j, double val)
#else
bool TechnosoftIposBase::setEncoderRaw(int j, double val)
#endif
{
    CHECK_JOINT(j);
    std::int32_t data = degreesToInternalUnits(val);

    if (!can->sdo()->download("Set actual position", data, 0x2081))
    {
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
        return yarp::dev::ReturnValue_error_method_failed;
#else
        return false;
#endif
    }

    lastEncoderRead->reset(data);
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
    return yarp::dev::ReturnValue_ok;
#else
    return true;
#endif
}

// -----------------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue TechnosoftIposBase::getEncoderRaw(int j, double * v)
#else
bool TechnosoftIposBase::getEncoderRaw(int j, double * v)
#endif
{
    CHECK_JOINT(j);
    std::int32_t temp = lastEncoderRead->queryPosition();
    *v = internalUnitsToDegrees(temp);
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
    return yarp::dev::ReturnValue_ok;
#else
    return true;
#endif
}

// -----------------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue TechnosoftIposBase::getEncoderSpeedRaw(int j, double * sp)
#else
bool TechnosoftIposBase::getEncoderSpeedRaw(int j, double * sp)
#endif
{
    CHECK_JOINT(j);
    double temp = lastEncoderRead->querySpeed();
    *sp = internalUnitsToDegrees(temp, 1);
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
    return yarp::dev::ReturnValue_ok;
#else
    return true;
#endif
}

// -----------------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue TechnosoftIposBase::getEncoderAccelerationRaw(int j, double * acc)
#else
bool TechnosoftIposBase::getEncoderAccelerationRaw(int j, double * acc)
#endif
{
    CHECK_JOINT(j);
    double temp = lastEncoderRead->queryAcceleration();
    *acc = internalUnitsToDegrees(temp, 2);
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
    return yarp::dev::ReturnValue_ok;
#else
    return true;
#endif
}

// -----------------------------------------------------------------------------------

#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
yarp::dev::ReturnValue TechnosoftIposBase::getEncoderTimedRaw(int j, double * enc, double * time)
#else
bool TechnosoftIposBase::getEncoderTimedRaw(int j, double * enc, double * time)
#endif
{
    CHECK_JOINT(j);
    std::int32_t temp = lastEncoderRead->queryPosition();
    *enc = internalUnitsToDegrees(temp);
    *time = lastEncoderRead->queryTime();
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
    return yarp::dev::ReturnValue_ok;
#else
    return true;
#endif
}

// -----------------------------------------------------------------------------------
