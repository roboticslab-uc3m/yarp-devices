// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "TechnosoftIpos.hpp"

#include <yarp/os/Log.h>

#include "LogComponent.hpp"

using namespace roboticslab;

// -----------------------------------------------------------------------------

bool TechnosoftIpos::getNumberOfMotorEncodersRaw(int * num)
{
    return getAxes(num);
}

// -----------------------------------------------------------------------------

bool TechnosoftIpos::resetMotorEncoderRaw(int m)
{
    yCITrace(IPOS, id(), "%d", m);
    CHECK_JOINT(m);
    return setMotorEncoderRaw(m, 0);
}

// -----------------------------------------------------------------------------

bool TechnosoftIpos::resetMotorEncodersRaw()
{
    return resetMotorEncoderRaw(0);
}

// -----------------------------------------------------------------------------

bool TechnosoftIpos::setMotorEncoderCountsPerRevolutionRaw(int m, double cpr)
{
    yCITrace(IPOS, id(), "%d %f", m, cpr);
    CHECK_JOINT(m);
    vars.encoderPulses = cpr;
    return true;
}

// -----------------------------------------------------------------------------

bool TechnosoftIpos::getMotorEncoderCountsPerRevolutionRaw(int m, double * cpr)
{
    yCITrace(IPOS, id(), "%d", m);
    CHECK_JOINT(m);
    *cpr = vars.encoderPulses;
    return true;
}

// -----------------------------------------------------------------------------

bool TechnosoftIpos::setMotorEncoderRaw(int m, double val)
{
    yCITrace(IPOS, id(), "%d %f", m, val);
    CHECK_JOINT(m);
    std::int32_t data = vars.reverse ? -val : val;

    if (!can->sdo()->download("Set actual position", data, 0X2081))
    {
        return false;
    }

    vars.lastEncoderRead->reset(data);
    return true;
}

// -----------------------------------------------------------------------------

bool TechnosoftIpos::setMotorEncodersRaw(const double * vals)
{
    return setMotorEncoderRaw(0, vals[0]);
}

// -----------------------------------------------------------------------------

bool TechnosoftIpos::getMotorEncoderRaw(int m, double * v)
{
    yCITrace(IPOS, id(), "%d", m);
    CHECK_JOINT(m);
    std::int32_t temp = vars.lastEncoderRead->queryPosition();
    *v = vars.reverse ? -temp : temp;
    return true;
}

// -----------------------------------------------------------------------------

bool TechnosoftIpos::getMotorEncodersRaw(double * encs)
{
    return getMotorEncoderSpeedRaw(0, &encs[0]);
}

// -----------------------------------------------------------------------------

bool TechnosoftIpos::getMotorEncoderTimedRaw(int m, double * enc, double * stamp)
{
    yCITrace(IPOS, id(), "%d", m);
    CHECK_JOINT(m);
    std::int32_t temp =  vars.lastEncoderRead->queryPosition();
    *enc = vars.reverse ? -temp : temp;
    *stamp = vars.lastEncoderRead->queryTime();
    return true;
}

// -----------------------------------------------------------------------------

bool TechnosoftIpos::getMotorEncodersTimedRaw(double * encs, double * stamps)
{
    return getMotorEncoderTimedRaw(0, &encs[0], &stamps[0]);
}

// -----------------------------------------------------------------------------

bool TechnosoftIpos::getMotorEncoderSpeedRaw(int m, double * sp)
{
    yCITrace(IPOS, id(), "%d", m);
    CHECK_JOINT(m);
    double temp = vars.lastEncoderRead->querySpeed();
    *sp = vars.reverse ? -temp : temp;
    return true;
}

// -----------------------------------------------------------------------------

bool TechnosoftIpos::getMotorEncoderSpeedsRaw(double * spds)
{
    return getMotorEncoderSpeedRaw(0, &spds[0]);
}

// -----------------------------------------------------------------------------

bool TechnosoftIpos::getMotorEncoderAccelerationRaw(int m, double * acc)
{
    yCITrace(IPOS, id(), "%d", m);
    CHECK_JOINT(m);
    double temp = vars.lastEncoderRead->queryAcceleration();
    *acc = vars.reverse ? -temp : temp;
    return true;
}

// -----------------------------------------------------------------------------

bool TechnosoftIpos::getMotorEncoderAccelerationsRaw(double * accs)
{
    return getMotorEncoderAccelerationRaw(0, &accs[0]);
}

// -----------------------------------------------------------------------------
