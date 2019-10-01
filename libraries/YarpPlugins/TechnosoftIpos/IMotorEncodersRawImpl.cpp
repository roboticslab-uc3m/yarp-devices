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
}

// -----------------------------------------------------------------------------

bool TechnosoftIpos::getMotorEncodersRaw(double * encs)
{
    CD_DEBUG("\n");
    return getMotorEncoderSpeedRaw(0, &encs[0]);
}

// -----------------------------------------------------------------------------

bool TechnosoftIpos::getMotorEncoderTimedRaw(int m, double * encs, double * stamp)
{
    CD_DEBUG("(%d)\n", m);
    CHECK_JOINT(m);
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
}

// -----------------------------------------------------------------------------

bool TechnosoftIpos::getMotorEncoderSpeedsRaw(double * spds)
{
    CD_DEBUG("\n");
    return getMotorEncoderSpeedRaw(0, &spds[0]);
}

// -----------------------------------------------------------------------------

bool TechnosoftIpos::getMotorEncoderAccelerationRaw(int m, double * spds)
{
    CD_DEBUG("(%d)\n", m);
    CHECK_JOINT(m);
}

// -----------------------------------------------------------------------------

bool TechnosoftIpos::getMotorEncoderAccelerationsRaw(double * vaccs)
{
    CD_DEBUG("\n");
    return getMotorEncoderAccelerationRaw(0, &vaccs[0]);
}

// -----------------------------------------------------------------------------
