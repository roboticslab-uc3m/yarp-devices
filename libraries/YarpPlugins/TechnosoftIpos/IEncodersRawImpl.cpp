// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "TechnosoftIposBase.hpp"

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
    CHECK_JOINT(j);
    return setEncoderRaw(j, 0.0);
}

// -----------------------------------------------------------------------------------

bool TechnosoftIposBase::setEncoderRaw(int j, double val)
{
    CHECK_JOINT(j);
    std::int32_t data = degreesToInternalUnits(val);

    if (!can->sdo()->download("Set actual position", data, 0x2081))
    {
        return false;
    }

    lastEncoderRead->reset(data);
    return true;
}

// -----------------------------------------------------------------------------------

bool TechnosoftIposBase::getEncoderRaw(int j, double * v)
{
    CHECK_JOINT(j);
    std::int32_t temp = lastEncoderRead->queryPosition();
    *v = internalUnitsToDegrees(temp);
    return true;
}

// -----------------------------------------------------------------------------------

bool TechnosoftIposBase::getEncoderSpeedRaw(int j, double * sp)
{
    CHECK_JOINT(j);
    double temp = lastEncoderRead->querySpeed();
    *sp = internalUnitsToDegrees(temp, 1);
    return true;
}

// -----------------------------------------------------------------------------------

bool TechnosoftIposBase::getEncoderAccelerationRaw(int j, double * acc)
{
    CHECK_JOINT(j);
    double temp = lastEncoderRead->queryAcceleration();
    *acc = internalUnitsToDegrees(temp, 2);
    return true;
}

// -----------------------------------------------------------------------------------

bool TechnosoftIposBase::getEncoderTimedRaw(int j, double * enc, double * time)
{
    CHECK_JOINT(j);
    std::int32_t temp = lastEncoderRead->queryPosition();
    *enc = internalUnitsToDegrees(temp);
    *time = lastEncoderRead->queryTime();
    return true;
}

// -----------------------------------------------------------------------------------
