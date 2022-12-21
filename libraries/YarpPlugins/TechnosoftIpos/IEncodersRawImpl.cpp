// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "TechnosoftIposBase.hpp"

#include <yarp/os/Log.h>

#include "LogComponent.hpp"

using namespace roboticslab;

// --------------------------------------------------------------------------------

bool TechnosoftIposBase::getAxes(int * ax)
{
    *ax = 1;
    return true;
}

// -----------------------------------------------------------------------------------

bool TechnosoftIposBase::resetEncoderRaw(int j)
{
    yCITrace(IPOS, id(), "%d", j);
    CHECK_JOINT(j);
    return setEncoderRaw(j, 0.0);
}

// -----------------------------------------------------------------------------------

bool TechnosoftIposBase::resetEncodersRaw()
{
    return resetEncoderRaw(0);
}

// -----------------------------------------------------------------------------------

bool TechnosoftIposBase::setEncoderRaw(int j, double val)
{
    yCITrace(IPOS, id(), "%d %f", j, val);
    CHECK_JOINT(j);
    std::int32_t data = degreesToInternalUnits(val);

    if (!can->sdo()->download("Set actual position", data, 0X2081))
    {
        return false;
    }

    lastEncoderRead->reset(data);
    return true;
}

// -----------------------------------------------------------------------------------

bool TechnosoftIposBase::setEncodersRaw(const double * vals)
{
    return setEncoderRaw(0, vals[0]);
}

// -----------------------------------------------------------------------------------

bool TechnosoftIposBase::getEncoderRaw(int j, double * v)
{
    yCITrace(IPOS, id(), "%d", j);
    CHECK_JOINT(j);
    std::int32_t temp = lastEncoderRead->queryPosition();
    *v = internalUnitsToDegrees(temp);
    return true;
}

// -----------------------------------------------------------------------------------

bool TechnosoftIposBase::getEncodersRaw(double * encs)
{
    return getEncoderRaw(0, &encs[0]);
}

// -----------------------------------------------------------------------------------

bool TechnosoftIposBase::getEncoderSpeedRaw(int j, double * sp)
{
    yCITrace(IPOS, id(), "%d", j);
    CHECK_JOINT(j);
    double temp = lastEncoderRead->querySpeed();
    *sp = internalUnitsToDegrees(temp, 1);
    return true;
}

// -----------------------------------------------------------------------------------

bool TechnosoftIposBase::getEncoderSpeedsRaw(double * spds)
{
    return getEncoderSpeedRaw(0, &spds[0]);
}

// -----------------------------------------------------------------------------------

bool TechnosoftIposBase::getEncoderAccelerationRaw(int j, double * acc)
{
    yCITrace(IPOS, id(), "%d", j);
    CHECK_JOINT(j);
    double temp = lastEncoderRead->queryAcceleration();
    *acc = internalUnitsToDegrees(temp, 2);
    return true;
}

// -----------------------------------------------------------------------------------

bool TechnosoftIposBase::getEncoderAccelerationsRaw(double * accs)
{
    return getEncoderAccelerationRaw(0, &accs[0]);
}

//---------------------------------------------------------------------------------------

bool TechnosoftIposBase::getEncoderTimedRaw(int j, double * enc, double * time)
{
    yCITrace(IPOS, id(), "%d", j);
    CHECK_JOINT(j);
    std::int32_t temp = lastEncoderRead->queryPosition();
    *enc = internalUnitsToDegrees(temp);
    *time = lastEncoderRead->queryTime();
    return true;
}

// -----------------------------------------------------------------------------------

bool TechnosoftIposBase::getEncodersTimedRaw(double * encs, double * times)
{
    return getEncoderTimedRaw(0, &encs[0], &times[0]);
}

// -----------------------------------------------------------------------------
