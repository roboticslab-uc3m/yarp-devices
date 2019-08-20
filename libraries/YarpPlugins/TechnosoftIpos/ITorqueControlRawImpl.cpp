// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "TechnosoftIpos.hpp"

// ############################# ITorqueControlRaw Related #############################

bool roboticslab::TechnosoftIpos::getRefTorquesRaw(double *t)
{
    CD_ERROR("Missing implementation\n");
    return false;
}

// -------------------------------------------------------------------------------------

bool roboticslab::TechnosoftIpos::getRefTorqueRaw(int j, double *t)
{
    CD_INFO("(%d)\n",j);
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

bool roboticslab::TechnosoftIpos::setRefTorquesRaw(const double *t)
{
    CD_ERROR("Missing implementation\n");
    return false;
}

// -------------------------------------------------------------------------------------

bool roboticslab::TechnosoftIpos::setRefTorqueRaw(int j, double t)
{
    CD_INFO("(%d, %f)\n", j, t);
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

bool roboticslab::TechnosoftIpos::getTorqueRaw(int j, double *t)
{
    //CD_INFO("(%d)\n",j);  //-- Too verbose in controlboardwrapper2 stream.
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

bool roboticslab::TechnosoftIpos::getTorquesRaw(double *t)
{
    CD_ERROR("Missing implementation\n");
    return false;
}

// -------------------------------------------------------------------------------------

bool roboticslab::TechnosoftIpos::getTorqueRangeRaw(int j, double *min, double *max)
{
    CD_INFO("(%d)\n",j);
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

bool roboticslab::TechnosoftIpos::getTorqueRangesRaw(double *min, double *max)
{
    CD_ERROR("Missing implementation\n");
    return false;
}

// -------------------------------------------------------------------------------------

bool roboticslab::TechnosoftIpos::getMotorTorqueParamsRaw(int j, yarp::dev::MotorTorqueParameters *params)
{
    CD_INFO("(%d)\n", j);
    CHECK_JOINT(j);

    params->bemf = 0.0;
    params->bemf_scale = 0.0;
    params->ktau = k;
    params->ktau_scale = 0.0;

    return true;
}

// -------------------------------------------------------------------------------------

bool roboticslab::TechnosoftIpos::setMotorTorqueParamsRaw(int j, const yarp::dev::MotorTorqueParameters params)
{
    CD_INFO("(%d)\n", j);
    CHECK_JOINT(j);
    k = params.ktau;
    return true;
}

// -------------------------------------------------------------------------------------
