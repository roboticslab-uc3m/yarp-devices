// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "TechnosoftIposBase.hpp"

#include <yarp/conf/version.h>
#include <yarp/os/Log.h>

#include "LogComponent.hpp"

using namespace roboticslab;

// -------------------------------------------------------------------------------------

bool TechnosoftIposBase::getRefTorqueRaw(int j, double * t)
{
    yCITrace(IPOS, id(), "%d", j);
    CHECK_JOINT(j);
    CHECK_MODE(VOCAB_CM_TORQUE);
    *t = vars.synchronousCommandTarget;
    return true;
}

// -------------------------------------------------------------------------------------

bool TechnosoftIposBase::getRefTorquesRaw(double * t)
{
    return getRefTorqueRaw(0, &t[0]);
}

// -------------------------------------------------------------------------------------

bool TechnosoftIposBase::setRefTorqueRaw(int j, double t)
{
    yCITrace(IPOS, id(), "%d %f", j, t);
    CHECK_JOINT(j);
    CHECK_MODE(VOCAB_CM_TORQUE);
    vars.synchronousCommandTarget = t;
    return true;
}

// -------------------------------------------------------------------------------------

bool TechnosoftIposBase::setRefTorquesRaw(const double * t)
{
    return setRefTorqueRaw(0, t[0]);
}

// -------------------------------------------------------------------------------------

bool TechnosoftIposBase::getTorqueRaw(int j, double * t)
{
    yCITrace(IPOS, id(), "%d", j);
    CHECK_JOINT(j);
    std::int16_t temp = vars.lastCurrentRead;
    double curr = vars.internalUnitsToCurrent(temp);
    *t = vars.currentToTorque(curr);
    return true;
}

// -------------------------------------------------------------------------------------

bool TechnosoftIposBase::getTorquesRaw(double * t)
{
    return getTorqueRaw(0, &t[0]);
}

// -------------------------------------------------------------------------------------

bool TechnosoftIposBase::getTorqueRangeRaw(int j, double * min, double * max)
{
    yCITrace(IPOS, id(), "%d", j);
    CHECK_JOINT(j);

    return can->sdo()->upload<std::uint16_t>("Current limit", [this, min, max](auto data)
        { double temp = vars.internalUnitsToPeakCurrent(data);
          *max = vars.currentToTorque(temp);
          *min = -(*max); },
        0x207F);
}

// -------------------------------------------------------------------------------------

bool TechnosoftIposBase::getTorqueRangesRaw(double * min, double * max)
{
    return getTorqueRangeRaw(0, &min[0], &max[0]);
}

// -------------------------------------------------------------------------------------

bool TechnosoftIposBase::getMotorTorqueParamsRaw(int j, yarp::dev::MotorTorqueParameters * params)
{
    yCITrace(IPOS, id(), "%d", j);
    CHECK_JOINT(j);

    params->bemf = 0.0;
    params->bemf_scale = 0.0;
    params->ktau = vars.k;
    params->ktau_scale = 0.0;
#if YARP_VERSION_COMPARE(>=, 3, 7, 0)
    params->viscousPos = 0.0;
    params->viscousNeg = 0.0;
    params->coulombPos = 0.0;
    params->coulombNeg = 0.0;
#endif

    return true;
}

// -------------------------------------------------------------------------------------

bool TechnosoftIposBase::setMotorTorqueParamsRaw(int j, const yarp::dev::MotorTorqueParameters params)
{
    yCITrace(IPOS, id(), "%d", j);
    CHECK_JOINT(j);
    vars.k = params.ktau;
    return true;
}

// -------------------------------------------------------------------------------------
