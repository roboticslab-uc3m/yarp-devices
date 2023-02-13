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
    *t = commandBuffer.getStoredCommand();
    return true;
}

// -------------------------------------------------------------------------------------

bool TechnosoftIposBase::setRefTorqueRaw(int j, double t)
{
    yCITrace(IPOS, id(), "%d %f", j, t);
    CHECK_JOINT(j);
    CHECK_MODE(VOCAB_CM_TORQUE);

    const auto state = limitSwitchState.load();

    if (state == INACTIVE || state == POSITIVE && t <= 0.0 || state == NEGATIVE && t >= 0.0)
    {
        commandBuffer.accept(t);
        return true;
    }
    else
    {
        return false;
    }
}

// -------------------------------------------------------------------------------------

bool TechnosoftIposBase::getTorqueRaw(int j, double * t)
{
    yCITrace(IPOS, id(), "%d", j);
    CHECK_JOINT(j);
    std::int16_t temp = lastCurrentRead;
    double curr = internalUnitsToCurrent(temp);
    *t = currentToTorque(curr);
    return true;
}

// -------------------------------------------------------------------------------------

bool TechnosoftIposBase::getTorqueRangeRaw(int j, double * min, double * max)
{
    yCITrace(IPOS, id(), "%d", j);
    CHECK_JOINT(j);

    return can->sdo()->upload<std::uint16_t>("Current limit", [this, min, max](auto data)
        { double temp = internalUnitsToPeakCurrent(data);
          *max = currentToTorque(temp);
          *min = -(*max); },
        0x207F);
}

// -------------------------------------------------------------------------------------

bool TechnosoftIposBase::getMotorTorqueParamsRaw(int j, yarp::dev::MotorTorqueParameters * params)
{
    yCITrace(IPOS, id(), "%d", j);
    CHECK_JOINT(j);

    params->bemf = 0.0;
    params->bemf_scale = 0.0;
    params->ktau = k;
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
    k = params.ktau;
    return true;
}

// -------------------------------------------------------------------------------------
