// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "LacqueyFetch.hpp"

// ######################### ITorqueControlRaw Related #########################

bool roboticslab::LacqueyFetch::getRefTorquesRaw(double *t)
{
    CD_ERROR("Missing implementation\n");
    return false;
}

// -----------------------------------------------------------------------------

bool roboticslab::LacqueyFetch::getRefTorqueRaw(int j, double *t)
{
    CD_INFO("\n");
    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::LacqueyFetch::setRefTorquesRaw(const double *t)
{
    CD_ERROR("Missing implementation\n");
    return false;
}

// -----------------------------------------------------------------------------

bool roboticslab::LacqueyFetch::setRefTorqueRaw(int j, double t)
{
    CD_INFO("\n");
    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::LacqueyFetch::getBemfParamRaw(int j, double *bemf)
{
    CD_INFO("\n");
    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::LacqueyFetch::setBemfParamRaw(int j, double bemf)
{
    CD_INFO("\n");
    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::LacqueyFetch::setTorquePidRaw(int j, const yarp::dev::Pid &pid)
{
    CD_INFO("\n");
    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::LacqueyFetch::getTorqueRaw(int j, double *t)
{
    //CD_INFO("\n");  //-- Too verbose in controlboardwrapper2 stream.
    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::LacqueyFetch::getTorquesRaw(double *t)
{
    CD_ERROR("Missing implementation\n");
    return false;
}

// -----------------------------------------------------------------------------

bool roboticslab::LacqueyFetch::getTorqueRangeRaw(int j, double *min, double *max)
{
    CD_INFO("\n");
    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::LacqueyFetch::getTorqueRangesRaw(double *min, double *max)
{
    CD_ERROR("Missing implementation\n");
    return false;
}

// -----------------------------------------------------------------------------

bool roboticslab::LacqueyFetch::setTorquePidsRaw(const yarp::dev::Pid *pids)
{
    CD_ERROR("Missing implementation\n");
    return false;
}

// -----------------------------------------------------------------------------

bool roboticslab::LacqueyFetch::setTorqueErrorLimitRaw(int j, double limit)
{
    CD_INFO("\n");
    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::LacqueyFetch::setTorqueErrorLimitsRaw(const double *limits)
{
    CD_ERROR("Missing implementation\n");
    return false;
}

// -----------------------------------------------------------------------------

bool roboticslab::LacqueyFetch::getTorqueErrorRaw(int j, double *err)
{
    CD_INFO("\n");
    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::LacqueyFetch::getTorqueErrorsRaw(double *errs)
{
    CD_ERROR("Missing implementation\n");
    return false;
}

// -----------------------------------------------------------------------------

bool roboticslab::LacqueyFetch::getTorquePidOutputRaw(int j, double *out)
{
    CD_INFO("\n");
    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::LacqueyFetch::getTorquePidOutputsRaw(double *outs)
{
    CD_ERROR("Missing implementation\n");
    return false;
}

// -----------------------------------------------------------------------------

bool roboticslab::LacqueyFetch::getTorquePidRaw(int j, yarp::dev::Pid *pid)
{
    CD_INFO("\n");
    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::LacqueyFetch::getTorquePidsRaw(yarp::dev::Pid *pids)
{
    CD_ERROR("Missing implementation\n");
    return false;
}

// -----------------------------------------------------------------------------

bool roboticslab::LacqueyFetch::getTorqueErrorLimitRaw(int j, double *limit)
{
    CD_INFO("\n");
    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::LacqueyFetch::getTorqueErrorLimitsRaw(double *limits)
{
    CD_ERROR("Missing implementation\n");
    return false;
}

// -----------------------------------------------------------------------------

bool roboticslab::LacqueyFetch::resetTorquePidRaw(int j)
{
    CD_INFO("\n");
    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::LacqueyFetch::disableTorquePidRaw(int j)
{
    CD_INFO("\n");
    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::LacqueyFetch::enableTorquePidRaw(int j)
{
    CD_INFO("\n");
    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::LacqueyFetch::setTorqueOffsetRaw(int j, double v)
{
    CD_INFO("\n");
    return true;
}
