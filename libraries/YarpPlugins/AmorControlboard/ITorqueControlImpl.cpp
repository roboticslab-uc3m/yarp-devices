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

bool roboticslab::AmorControlboard::getBemfParam(int j, double *bemf)
{
    CD_DEBUG("(%d)\n", j);

    if (!indexWithinRange(j))
    {
        return false;
    }

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

bool roboticslab::AmorControlboard::setTorquePid(int j, const yarp::dev::Pid &pid)
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

bool roboticslab::AmorControlboard::setTorquePids(const yarp::dev::Pid *pids)
{
    CD_DEBUG("\n");
    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::AmorControlboard::setTorqueErrorLimit(int j, double limit)
{
    CD_DEBUG("(%d, %f)\n", j, limit);

    if (!indexWithinRange(j))
    {
        return false;
    }

    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::AmorControlboard::setTorqueErrorLimits(const double *limits)
{
    CD_DEBUG("\n");
    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::AmorControlboard::getTorqueError(int j, double *err)
{
    CD_DEBUG("(%d)\n", j);

    if (!indexWithinRange(j))
    {
        return false;
    }

    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::AmorControlboard::getTorqueErrors(double *errs)
{
    CD_DEBUG("\n");
    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::AmorControlboard::getTorquePidOutput(int j, double *out)
{
    CD_DEBUG("(%d)\n", j);

    if (!indexWithinRange(j))
    {
        return false;
    }

    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::AmorControlboard::getTorquePidOutputs(double *outs)
{
    CD_DEBUG("\n");
    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::AmorControlboard::getTorquePid(int j, yarp::dev::Pid *pid)
{
    CD_DEBUG("(%d)\n", j);

    if (!indexWithinRange(j))
    {
        return false;
    }

    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::AmorControlboard::getTorquePids(yarp::dev::Pid *pids)
{
    CD_DEBUG("\n");
    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::AmorControlboard::getTorqueErrorLimit(int j, double *limit)
{
    CD_DEBUG("(%d)\n", j);

    if (!indexWithinRange(j))
    {
        return false;
    }

    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::AmorControlboard::getTorqueErrorLimits(double *limits)
{
    CD_DEBUG("\n");
    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::AmorControlboard::resetTorquePid(int j)
{
    CD_DEBUG("(%d)\n", j);

    if (!indexWithinRange(j))
    {
        return false;
    }

    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::AmorControlboard::disableTorquePid(int j)
{
    CD_DEBUG("(%d)\n", j);

    if (!indexWithinRange(j))
    {
        return false;
    }

    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::AmorControlboard::enableTorquePid(int j)
{
    CD_DEBUG("(%d)\n", j);

    if (!indexWithinRange(j))
    {
        return false;
    }

    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::AmorControlboard::setTorqueOffset(int j, double v)
{
    CD_DEBUG("(%d, %f)\n", j, v);

    if (!indexWithinRange(j))
    {
        return false;
    }

    return true;
}

// -----------------------------------------------------------------------------
