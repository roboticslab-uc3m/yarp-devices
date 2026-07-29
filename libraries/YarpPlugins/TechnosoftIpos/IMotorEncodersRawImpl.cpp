// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "TechnosoftIposBase.hpp"

using namespace roboticslab;

// -----------------------------------------------------------------------------

bool TechnosoftIposBase::getNumberOfMotorEncodersRaw(int * num)
{
    return getAxes(num);
}

// -----------------------------------------------------------------------------

bool TechnosoftIposBase::resetMotorEncoderRaw(int m)
{
    CHECK_JOINT(m);
    return setMotorEncoderRaw(m, 0);
}

// -----------------------------------------------------------------------------

bool TechnosoftIposBase::setMotorEncoderCountsPerRevolutionRaw(int m, double cpr)
{
    CHECK_JOINT(m);
    encoderPulses = cpr;
    return true;
}

// -----------------------------------------------------------------------------

bool TechnosoftIposBase::getMotorEncoderCountsPerRevolutionRaw(int m, double * cpr)
{
    CHECK_JOINT(m);
    *cpr = encoderPulses;
    return true;
}

// -----------------------------------------------------------------------------

bool TechnosoftIposBase::setMotorEncoderRaw(int m, double val)
{
    CHECK_JOINT(m);
    std::int32_t data = params.m_reverse ? -val : val;

    if (!can->sdo()->download("Set actual position", data, 0x2081))
    {
        return false;
    }

    lastEncoderRead->reset(data);
    return true;
}

// -----------------------------------------------------------------------------

bool TechnosoftIposBase::getMotorEncoderRaw(int m, double * v)
{
    CHECK_JOINT(m);
    std::int32_t temp = lastEncoderRead->queryPosition();
    *v = params.m_reverse ? -temp : temp;
    return true;
}

// -----------------------------------------------------------------------------

bool TechnosoftIposBase::getMotorEncoderTimedRaw(int m, double * enc, double * stamp)
{
    CHECK_JOINT(m);
    std::int32_t temp =  lastEncoderRead->queryPosition();
    *enc = params.m_reverse ? -temp : temp;
    *stamp = lastEncoderRead->queryTime();
    return true;
}

// -----------------------------------------------------------------------------

bool TechnosoftIposBase::getMotorEncoderSpeedRaw(int m, double * sp)
{
    CHECK_JOINT(m);
    double temp = lastEncoderRead->querySpeed();
    *sp = params.m_reverse ? -temp : temp;
    return true;
}

// -----------------------------------------------------------------------------

bool TechnosoftIposBase::getMotorEncoderAccelerationRaw(int m, double * acc)
{
    CHECK_JOINT(m);
    double temp = lastEncoderRead->queryAcceleration();
    *acc = params.m_reverse ? -temp : temp;
    return true;
}

// -----------------------------------------------------------------------------
