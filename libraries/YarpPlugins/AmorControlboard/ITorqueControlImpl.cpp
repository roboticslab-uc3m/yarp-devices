// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "AmorControlboard.hpp"

// ------------------ ITorqueControl related -----------------------------------------

bool roboticslab::AmorControlboard::getRefTorques(double *t)
{
    CD_DEBUG("\n");
    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::AmorControlboard::getRefTorque(int j, double *t)
{
    CD_DEBUG("(%d)\n", j);

    if (!indexWithinRange(j))
    {
        return false;
    }

    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::AmorControlboard::setRefTorques(const double *t)
{
    CD_DEBUG("\n");
    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::AmorControlboard::setRefTorque(int j, double t)
{
    CD_DEBUG("(%d, %f)\n", j, t);

    if (!indexWithinRange(j))
    {
        return false;
    }

    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::AmorControlboard::setRefTorques(const int n_joint, const int *joints, const double *t)
{
    CD_DEBUG("(%d)\n", n_joint);

    if (!batchWithinRange(n_joint))
    {
        return false;
    }

    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::AmorControlboard::getMotorTorqueParams(int j, yarp::dev::MotorTorqueParameters *params)
{
    CD_DEBUG("(%d)\n", j);

    if (!indexWithinRange(j))
    {
        return false;
    }

    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::AmorControlboard::setMotorTorqueParams(int j, const yarp::dev::MotorTorqueParameters params)
{
    CD_DEBUG("(%d)\n", j);

    if (!indexWithinRange(j))
    {
        return false;
    }

    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::AmorControlboard::getTorque(int j, double *t)
{
    //CD_DEBUG("(%d)\n", j);

    if (!indexWithinRange(j))
    {
        return false;
    }

    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::AmorControlboard::getTorques(double *t)
{
    CD_DEBUG("\n");
    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::AmorControlboard::getTorqueRange(int j, double *min, double *max)
{
    CD_DEBUG("(%d)\n", j);

    if (!indexWithinRange(j))
    {
        return false;
    }

    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::AmorControlboard::getTorqueRanges(double *min, double *max)
{
    CD_DEBUG("\n");
    return true;
}

// -----------------------------------------------------------------------------

#if YARP_VERSION_MAJOR != 3
bool roboticslab::AmorControlboard::getBemfParam(int j, double *bemf)
{
    CD_DEBUG("(%d)\n", j);

    if (!indexWithinRange(j))
    {
        return false;
    }

    yarp::dev::MotorTorqueParameters params;

    if (!getMotorTorqueParams(j, &params))
    {
        return false;
    }

    *bemf = params.bemf;

    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::AmorControlboard::setBemfParam(int j, double bemf)
{
    CD_DEBUG("(%d, %f)\n", j, bemf);

    if (!indexWithinRange(j))
    {
        return false;
    }

    yarp::dev::MotorTorqueParameters params;

    if (!getMotorTorqueParams(j, &params))
    {
        return false;
    }

    params.bemf = bemf;

    return setMotorTorqueParams(j, params);
}

// -----------------------------------------------------------------------------
#endif // YARP_VERSION_MAJOR != 3
