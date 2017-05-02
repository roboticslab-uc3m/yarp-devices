// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "CuiAbsolute.hpp"

bool teo::CuiAbsolute::getRefTorquesRaw(double *t)
{
    CD_ERROR("\n");
    return false;
}

bool teo::CuiAbsolute::getRefTorqueRaw(int j, double *t)
{
    CD_INFO("\n");
    return true;
}

bool teo::CuiAbsolute::setRefTorquesRaw(const double *t)
{
    CD_ERROR("\n");
    return false;
}

bool teo::CuiAbsolute::setRefTorqueRaw(int j, double t)
{
    CD_INFO("\n");
    return true;
}

bool teo::CuiAbsolute::getBemfParamRaw(int j, double *bemf)
{
    CD_INFO("\n");
    return true;
}

bool teo::CuiAbsolute::setBemfParamRaw(int j, double bemf)
{
    CD_INFO("\n");
    return true;
}

bool teo::CuiAbsolute::setTorquePidRaw(int j, const yarp::dev::Pid &pid)
{
    CD_INFO("\n");
    return true;
}

bool teo::CuiAbsolute::getTorqueRaw(int j, double *t)
{
    //CD_INFO("\n");  //-- Too verbose in controlboardwrapper2 stream.
    return true;
}

bool teo::CuiAbsolute::getTorquesRaw(double *t)
{
    CD_ERROR("\n");
    return false;
}

bool teo::CuiAbsolute::getTorqueRangeRaw(int j, double *min, double *max)
{
    CD_INFO("\n");
    return true;
}

bool teo::CuiAbsolute::getTorqueRangesRaw(double *min, double *max)
{
    CD_ERROR("\n");
    return false;
}

bool teo::CuiAbsolute::setTorquePidsRaw(const yarp::dev::Pid *pids)
{
    CD_ERROR("\n");
    return false;
}

bool teo::CuiAbsolute::setTorqueErrorLimitRaw(int j, double limit)
{
    CD_INFO("\n");
    return true;
}

bool teo::CuiAbsolute::setTorqueErrorLimitsRaw(const double *limits)
{
    CD_ERROR("\n");
    return false;
}

bool teo::CuiAbsolute::getTorqueErrorRaw(int j, double *err)
{
    CD_INFO("\n");
    return true;
}

bool teo::CuiAbsolute::getTorqueErrorsRaw(double *errs)
{
    CD_ERROR("\n");
    return false;
}

bool teo::CuiAbsolute::getTorquePidOutputRaw(int j, double *out)
{
    CD_INFO("\n");
    return true;
}

bool teo::CuiAbsolute::getTorquePidOutputsRaw(double *outs)
{
    CD_ERROR("\n");
    return false;
}

bool teo::CuiAbsolute::getTorquePidRaw(int j, yarp::dev::Pid *pid)
{
    CD_INFO("\n");
    return true;
}

bool teo::CuiAbsolute::getTorquePidsRaw(yarp::dev::Pid *pids)
{
    CD_ERROR("\n");
    return false;
}

bool teo::CuiAbsolute::getTorqueErrorLimitRaw(int j, double *limit)
{
    CD_INFO("\n");
    return true;
}

bool teo::CuiAbsolute::getTorqueErrorLimitsRaw(double *limits)
{
    CD_ERROR("\n");
    return false;
}

bool teo::CuiAbsolute::resetTorquePidRaw(int j)
{
    CD_INFO("\n");
    return true;
}

bool teo::CuiAbsolute::disableTorquePidRaw(int j)
{
    CD_INFO("\n");
    return true;
}

bool teo::CuiAbsolute::enableTorquePidRaw(int j)
{
    CD_INFO("\n");
    return true;
}

bool teo::CuiAbsolute::setTorqueOffsetRaw(int j, double v)
{
    CD_INFO("\n");
    return true;
}

