// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "TechnosoftIposBase.hpp"

#include <yarp/os/Log.h>

#include "LogComponent.hpp"

using namespace roboticslab;

// -----------------------------------------------------------------------------

bool TechnosoftIposBase::getNumberOfMotorEncodersRaw(int * num)
{
    return getAxes(num);
}

// -----------------------------------------------------------------------------

bool TechnosoftIposBase::resetMotorEncoderRaw(int m)
{
    yCITrace(IPOS, id(), "%d", m);
    CHECK_JOINT(m);
    return setMotorEncoderRaw(m, 0);
}

// -----------------------------------------------------------------------------

bool TechnosoftIposBase::resetMotorEncodersRaw()
{
    return resetMotorEncoderRaw(0);
}

// -----------------------------------------------------------------------------

bool TechnosoftIposBase::setMotorEncoderCountsPerRevolutionRaw(int m, double cpr)
{
    yCITrace(IPOS, id(), "%d %f", m, cpr);
    CHECK_JOINT(m);
    encoderPulses = cpr;
    return true;
}

// -----------------------------------------------------------------------------

bool TechnosoftIposBase::getMotorEncoderCountsPerRevolutionRaw(int m, double * cpr)
{
    yCITrace(IPOS, id(), "%d", m);
    CHECK_JOINT(m);
    *cpr = encoderPulses;
    return true;
}

// -----------------------------------------------------------------------------

bool TechnosoftIposBase::setMotorEncoderRaw(int m, double val)
{
    yCITrace(IPOS, id(), "%d %f", m, val);
    CHECK_JOINT(m);
    std::int32_t data = reverse ? -val : val;

    if (!can->sdo()->download("Set actual position", data, 0X2081))
    {
        return false;
    }

    lastEncoderRead->reset(data);
    return true;
}

// -----------------------------------------------------------------------------

bool TechnosoftIposBase::setMotorEncodersRaw(const double * vals)
{
    return setMotorEncoderRaw(0, vals[0]);
}

// -----------------------------------------------------------------------------

bool TechnosoftIposBase::getMotorEncoderRaw(int m, double * v)
{
    yCITrace(IPOS, id(), "%d", m);
    CHECK_JOINT(m);
    std::int32_t temp = lastEncoderRead->queryPosition();
    *v = reverse ? -temp : temp;
    return true;
}

// -----------------------------------------------------------------------------

bool TechnosoftIposBase::getMotorEncodersRaw(double * encs)
{
    return getMotorEncoderSpeedRaw(0, &encs[0]);
}

// -----------------------------------------------------------------------------

bool TechnosoftIposBase::getMotorEncoderTimedRaw(int m, double * enc, double * stamp)
{
    yCITrace(IPOS, id(), "%d", m);
    CHECK_JOINT(m);
    std::int32_t temp =  lastEncoderRead->queryPosition();
    *enc = reverse ? -temp : temp;
    *stamp = lastEncoderRead->queryTime();
    return true;
}

// -----------------------------------------------------------------------------

bool TechnosoftIposBase::getMotorEncodersTimedRaw(double * encs, double * stamps)
{
    return getMotorEncoderTimedRaw(0, &encs[0], &stamps[0]);
}

// -----------------------------------------------------------------------------

bool TechnosoftIposBase::getMotorEncoderSpeedRaw(int m, double * sp)
{
    yCITrace(IPOS, id(), "%d", m);
    CHECK_JOINT(m);
    double temp = lastEncoderRead->querySpeed();
    *sp = reverse ? -temp : temp;
    return true;
}

// -----------------------------------------------------------------------------

bool TechnosoftIposBase::getMotorEncoderSpeedsRaw(double * spds)
{
    return getMotorEncoderSpeedRaw(0, &spds[0]);
}

// -----------------------------------------------------------------------------

bool TechnosoftIposBase::getMotorEncoderAccelerationRaw(int m, double * acc)
{
    yCITrace(IPOS, id(), "%d", m);
    CHECK_JOINT(m);
    double temp = lastEncoderRead->queryAcceleration();
    *acc = reverse ? -temp : temp;
    return true;
}

// -----------------------------------------------------------------------------

bool TechnosoftIposBase::getMotorEncoderAccelerationsRaw(double * accs)
{
    return getMotorEncoderAccelerationRaw(0, &accs[0]);
}

// -----------------------------------------------------------------------------
