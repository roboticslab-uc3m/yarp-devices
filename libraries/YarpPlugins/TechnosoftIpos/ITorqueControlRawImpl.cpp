// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "TechnosoftIpos.hpp"

#include <ColorDebug.h>

using namespace roboticslab;

// -------------------------------------------------------------------------------------

bool TechnosoftIpos::getRefTorqueRaw(int j, double * t)
{
    CD_DEBUG("(%d)\n", j);
    CHECK_JOINT(j);

    double curr;

    if (!getRefCurrentRaw(j, &curr))
    {
        CD_ERROR("getRefCurrentRaw() failed.\n");
        return false;
    }

    *t = currentToTorque(curr);
    return true;
}

// -------------------------------------------------------------------------------------

bool TechnosoftIpos::getRefTorquesRaw(double * t)
{
    CD_DEBUG("\n");
    return getRefTorqueRaw(0, &t[0]);
}

// -------------------------------------------------------------------------------------

bool TechnosoftIpos::setRefTorqueRaw(int j, double t)
{
    CD_DEBUG("(%d, %f)\n", j, t);
    CHECK_JOINT(j);

    double curr = torqueToCurrent(t);

    if (!setRefCurrentRaw(j, curr))
    {
        CD_ERROR("setRefCurrentRaw() failed.\n");
        return false;
    }

    return true;
}

// -------------------------------------------------------------------------------------

bool TechnosoftIpos::setRefTorquesRaw(const double * t)
{
    CD_DEBUG("\n");
    return setRefTorqueRaw(0, t[0]);
}

// -------------------------------------------------------------------------------------

bool TechnosoftIpos::getTorqueRaw(int j, double * t)
{
    //CD_DEBUG("(%d)\n", j); //-- Too verbose in controlboardwrapper2 stream.
    CHECK_JOINT(j);

    double curr;

    if (!getCurrentRaw(j, &curr))
    {
        CD_ERROR("getCurrentRaw() failed.\n");
        return false;
    }

    *t = currentToTorque(curr);
    return true;
}

// -------------------------------------------------------------------------------------

bool TechnosoftIpos::getTorquesRaw(double * t)
{
    CD_DEBUG("\n");
    return getTorqueRaw(0, &t[0]);
}

// -------------------------------------------------------------------------------------

bool TechnosoftIpos::getTorqueRangeRaw(int j, double * min, double * max)
{
    CD_DEBUG("(%d)\n", j);
    CHECK_JOINT(j);

    double minCurrent, maxCurrent;

    if (!getCurrentRangeRaw(j, &minCurrent, &maxCurrent))
    {
        CD_ERROR("getCurrentRangeRaw() failed.\n");
        return false;
    }

    *max = currentToTorque(maxCurrent);
    *min = -(*max);

    return true;
}

// -------------------------------------------------------------------------------------

bool TechnosoftIpos::getTorqueRangesRaw(double * min, double * max)
{
    CD_DEBUG("\n");
    return getTorqueRangeRaw(0, &min[0], &max[0]);
}

// -------------------------------------------------------------------------------------

bool TechnosoftIpos::getMotorTorqueParamsRaw(int j, yarp::dev::MotorTorqueParameters * params)
{
    CD_DEBUG("(%d)\n", j);
    CHECK_JOINT(j);

    params->bemf = 0.0;
    params->bemf_scale = 0.0;
    params->ktau = k;
    params->ktau_scale = 0.0;

    return true;
}

// -------------------------------------------------------------------------------------

bool TechnosoftIpos::setMotorTorqueParamsRaw(int j, const yarp::dev::MotorTorqueParameters params)
{
    CD_DEBUG("(%d)\n", j);
    CHECK_JOINT(j);
    k = params.ktau;
    return true;
}

// -------------------------------------------------------------------------------------
