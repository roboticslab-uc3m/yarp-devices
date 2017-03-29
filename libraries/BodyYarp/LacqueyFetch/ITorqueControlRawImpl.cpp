// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "LacqueyFetch.hpp"

// ------- ITorqueControlRaw Related -------

bool teo::LacqueyFetch::setTorqueModeRaw()
{
    CD_ERROR("\n");
    return false;
}

bool teo::LacqueyFetch::getRefTorquesRaw(double *t)
{
    CD_ERROR("\n");
    return false;
}

bool teo::LacqueyFetch::getRefTorqueRaw(int j, double *t)
{
    CD_INFO("\n");
    return true;
}

bool teo::LacqueyFetch::setRefTorquesRaw(const double *t)
{
    CD_ERROR("\n");
    return false;
}

bool teo::LacqueyFetch::setRefTorqueRaw(int j, double t)
{
    CD_INFO("\n");
    return true;
}

bool teo::LacqueyFetch::getBemfParamRaw(int j, double *bemf)
{
    CD_INFO("\n");
    return true;
}

bool teo::LacqueyFetch::setBemfParamRaw(int j, double bemf)
{
    CD_INFO("\n");
    return true;
}

bool teo::LacqueyFetch::setTorquePidRaw(int j, const yarp::dev::Pid &pid)
{
    CD_INFO("\n");
    return true;
}

bool teo::LacqueyFetch::getTorqueRaw(int j, double *t)
{
    //CD_INFO("\n");  //-- Too verbose in controlboardwrapper2 stream.
    return true;
}

bool teo::LacqueyFetch::getTorquesRaw(double *t)
{
    CD_ERROR("\n");
    return false;
}

bool teo::LacqueyFetch::getTorqueRangeRaw(int j, double *min, double *max)
{
    CD_INFO("\n");
    return true;
}

bool teo::LacqueyFetch::getTorqueRangesRaw(double *min, double *max)
{
    CD_ERROR("\n");
    return false;
}

bool teo::LacqueyFetch::setTorquePidsRaw(const yarp::dev::Pid *pids)
{
    CD_ERROR("\n");
    return false;
}

bool teo::LacqueyFetch::setTorqueErrorLimitRaw(int j, double limit)
{
    CD_INFO("\n");
    return true;
}

bool teo::LacqueyFetch::setTorqueErrorLimitsRaw(const double *limits)
{
    CD_ERROR("\n");
    return false;
}

bool teo::LacqueyFetch::getTorqueErrorRaw(int j, double *err)
{
    CD_INFO("\n");
    return true;
}

bool teo::LacqueyFetch::getTorqueErrorsRaw(double *errs)
{
    CD_ERROR("\n");
    return false;
}

bool teo::LacqueyFetch::getTorquePidOutputRaw(int j, double *out)
{
    CD_INFO("\n");
    return true;
}

bool teo::LacqueyFetch::getTorquePidOutputsRaw(double *outs)
{
    CD_ERROR("\n");
    return false;
}

bool teo::LacqueyFetch::getTorquePidRaw(int j, yarp::dev::Pid *pid)
{
    CD_INFO("\n");
    return true;
}

bool teo::LacqueyFetch::getTorquePidsRaw(yarp::dev::Pid *pids)
{
    CD_ERROR("\n");
    return false;
}

bool teo::LacqueyFetch::getTorqueErrorLimitRaw(int j, double *limit)
{
    CD_INFO("\n");
    return true;
}

bool teo::LacqueyFetch::getTorqueErrorLimitsRaw(double *limits)
{
    CD_ERROR("\n");
    return false;
}

bool teo::LacqueyFetch::resetTorquePidRaw(int j)
{
    CD_INFO("\n");
    return true;
}

bool teo::LacqueyFetch::disableTorquePidRaw(int j)
{
    CD_INFO("\n");
    return true;
}

bool teo::LacqueyFetch::enableTorquePidRaw(int j)
{
    CD_INFO("\n");
    return true;
}

bool teo::LacqueyFetch::setTorqueOffsetRaw(int j, double v)
{
    CD_INFO("\n");
    return true;
}
