// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "FakeControlboard.hpp"

#include <ColorDebug.hpp>

// ------------------- IForceControl Related ------------------------------------

bool roboticslab::FakeControlboard::getRefTorques(double *t)
{
    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::FakeControlboard::getRefTorque(int j, double *t)
{
    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::FakeControlboard::setRefTorques(const double *t)
{
    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::FakeControlboard::setRefTorque(int j, double t)
{
    CD_DEBUG("joint: %d, refTorque: %f.\n", j, t);
    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::FakeControlboard::getBemfParam(int j, double *bemf)
{
    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::FakeControlboard::setBemfParam(int j, double bemf)
{
    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::FakeControlboard::setTorquePid(int j, const yarp::dev::Pid &pid)
{
    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::FakeControlboard::getTorque(int j, double *t)
{
    //CD_DEBUG("joint: %d.\n",j);  //-- Way too verbose
    *t = 0;
    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::FakeControlboard::getTorques(double *t)
{
    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::FakeControlboard::getTorqueRange(int j, double *min, double *max)
{
    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::FakeControlboard::getTorqueRanges(double *min, double *max)
{
    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::FakeControlboard::setTorquePids(const yarp::dev::Pid *pids)
{
    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::FakeControlboard::setTorqueErrorLimit(int j, double limit)
{
    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::FakeControlboard::setTorqueErrorLimits(const double *limits)
{
    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::FakeControlboard::getTorqueError(int j, double *err)
{
    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::FakeControlboard::getTorqueErrors(double *errs)
{
    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::FakeControlboard::getTorquePidOutput(int j, double *out)
{
    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::FakeControlboard::getTorquePidOutputs(double *outs)
{
    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::FakeControlboard::getTorquePid(int j, yarp::dev::Pid *pid)
{
    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::FakeControlboard::getTorquePids(yarp::dev::Pid *pids)
{
    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::FakeControlboard::getTorqueErrorLimit(int j, double *limit)
{
    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::FakeControlboard::getTorqueErrorLimits(double *limits)
{
    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::FakeControlboard::resetTorquePid(int j)
{
    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::FakeControlboard::disableTorquePid(int j)
{
    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::FakeControlboard::enableTorquePid(int j)
{
    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::FakeControlboard::setTorqueOffset(int j, double v)
{
    return true;
}

// -----------------------------------------------------------------------------
