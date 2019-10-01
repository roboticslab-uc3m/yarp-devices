// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "TechnosoftIpos.hpp"

#include <ColorDebug.h>

using namespace roboticslab;

// -----------------------------------------------------------------------------

bool TechnosoftIpos::getNumberOfMotorEncodersRaw(int * num)
{
    return getAxes(num);
}

// -----------------------------------------------------------------------------

bool TechnosoftIpos::resetMotorEncoderRaw(int m)
{
    CD_DEBUG("(%d)\n", m);
    CHECK_JOINT(m);
    return setMotorEncoderRaw(m, 0);
}

// -----------------------------------------------------------------------------

bool TechnosoftIpos::resetMotorEncodersRaw()
{
    CD_DEBUG("\n");
    return resetMotorEncoderRaw(0);
}

// -----------------------------------------------------------------------------

bool TechnosoftIpos::setMotorEncoderCountsPerRevolutionRaw(int m, double cpr)
{
    CD_DEBUG("(%d, %f)\n", m, cpr);
    CHECK_JOINT(m);
    vars.encoderPulses = cpr;
    return true;
}

// -----------------------------------------------------------------------------

bool TechnosoftIpos::getMotorEncoderCountsPerRevolutionRaw(int m, double * cpr)
{
    CD_DEBUG("(%d)\n", m);
    CHECK_JOINT(m);
    *cpr = vars.encoderPulses;
    return true;
}

// -----------------------------------------------------------------------------

bool TechnosoftIpos::setMotorEncoderRaw(int m, double val)
{
    CD_DEBUG("(%d, %f)\n", m, val);
    CHECK_JOINT(m);
    std::int32_t data = vars.reverse ? -val : val;

    if (!can->sdo()->download("Set actual position", data, 0X2081))
    {
        return false;
    }

    vars.lastEncoderRead.reset(data);
    return true;
}

// -----------------------------------------------------------------------------

bool TechnosoftIpos::setMotorEncodersRaw(const double * vals)
{
    CD_DEBUG("\n");
    return setMotorEncoderRaw(0, vals[0]);
}

// -----------------------------------------------------------------------------

bool TechnosoftIpos::getMotorEncoderRaw(int m, double * v)
{
    CD_DEBUG("(%d)\n", m);
    CHECK_JOINT(m);
    std::int32_t temp = vars.lastEncoderRead.queryPosition();
    *v = vars.reverse ? -temp : temp;
    return true;
}

// -----------------------------------------------------------------------------

bool TechnosoftIpos::getMotorEncodersRaw(double * encs)
{
    CD_DEBUG("\n");
    return getMotorEncoderSpeedRaw(0, &encs[0]);
}

// -----------------------------------------------------------------------------

bool TechnosoftIpos::getMotorEncoderTimedRaw(int m, double * enc, double * stamp)
{
    CD_DEBUG("(%d)\n", m);
    CHECK_JOINT(m);
    std::int32_t temp =  vars.lastEncoderRead.queryPosition();
    *enc = vars.reverse ? -temp : temp;
    *stamp = vars.lastEncoderRead.queryTime();
    return true;
}

// -----------------------------------------------------------------------------

bool TechnosoftIpos::getMotorEncodersTimedRaw(double * encs, double * stamps)
{
    CD_DEBUG("\n");
    return getMotorEncoderTimedRaw(0, &encs[0], &stamps[0]);
}

// -----------------------------------------------------------------------------

bool TechnosoftIpos::getMotorEncoderSpeedRaw(int m, double * sp)
{
    CD_DEBUG("(%d)\n", m);
    CHECK_JOINT(m);
    double temp = vars.lastEncoderRead.querySpeed();
    *sp = vars.reverse ? -temp : temp;
    return true;
}

// -----------------------------------------------------------------------------

bool TechnosoftIpos::getMotorEncoderSpeedsRaw(double * spds)
{
    CD_DEBUG("\n");
    return getMotorEncoderSpeedRaw(0, &spds[0]);
}

// -----------------------------------------------------------------------------

bool TechnosoftIpos::getMotorEncoderAccelerationRaw(int m, double * acc)
{
    CD_DEBUG("(%d)\n", m);
    CHECK_JOINT(m);
    double temp = vars.lastEncoderRead.queryAcceleration();
    *acc = vars.reverse ? -temp : temp;
    return true;
}

// -----------------------------------------------------------------------------

bool TechnosoftIpos::getMotorEncoderAccelerationsRaw(double * accs)
{
    CD_DEBUG("\n");
    return getMotorEncoderAccelerationRaw(0, &accs[0]);
}

// -----------------------------------------------------------------------------
