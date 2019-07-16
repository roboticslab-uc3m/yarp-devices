// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "TechnosoftIpos.hpp"

#include <cmath>

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

    //-- Check index within range
    if ( j != 0 ) return false;

    refTorqueSemaphore.wait();
    *t = refTorque;
    refTorqueSemaphore.post();

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

    double curr = t / (std::abs(tr) * k);

    if (!setRefCurrentRaw(j, curr))
    {
        CD_ERROR("setRefCurrentRaw() failed.\n");
        return false;
    }

    refTorqueSemaphore.wait();
    refTorque = t;
    refTorqueSemaphore.post();

    return true;
}

// -------------------------------------------------------------------------------------

bool roboticslab::TechnosoftIpos::getTorqueRaw(int j, double *t)
{
    //CD_INFO("(%d)\n",j);  //-- Too verbose in controlboardwrapper2 stream.

    double curr;

    if (!getCurrentRaw(j, &curr))
    {
        CD_ERROR("getCurrentRaw() failed.\n");
        return false;
    }

    *t = curr * std::abs(tr) * k;

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

    //-- Check index within range
    if ( j != 0 ) return false;

    CD_WARNING("Not implemented yet (TechnosoftIpos).\n");

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

    //-- Check index within range
    if ( j != 0 ) return false;

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

    //-- Check index within range
    if ( j != 0 ) return false;

    k = params.ktau;

    return true;
}

// -------------------------------------------------------------------------------------
