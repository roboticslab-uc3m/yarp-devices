// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "CuiAbsolute.hpp"

bool roboticslab::CuiAbsolute::getRefTorquesRaw(double *t)
{
    CD_ERROR("\n");
    return false;
}

bool roboticslab::CuiAbsolute::getRefTorqueRaw(int j, double *t)
{
    CD_INFO("\n");
    return true;
}

bool roboticslab::CuiAbsolute::setRefTorquesRaw(const double *t)
{
    CD_ERROR("\n");
    return false;
}

bool roboticslab::CuiAbsolute::setRefTorqueRaw(int j, double t)
{
    CD_INFO("\n");
    return true;
}

bool roboticslab::CuiAbsolute::getTorqueRaw(int j, double *t)
{
    //CD_INFO("\n");  //-- Too verbose in controlboardwrapper2 stream.
    return true;
}

bool roboticslab::CuiAbsolute::getTorquesRaw(double *t)
{
    CD_ERROR("\n");
    return false;
}

bool roboticslab::CuiAbsolute::getTorqueRangeRaw(int j, double *min, double *max)
{
    CD_INFO("\n");
    return true;
}

bool roboticslab::CuiAbsolute::getTorqueRangesRaw(double *min, double *max)
{
    CD_ERROR("\n");
    return false;
}

#if YARP_VERSION_MAJOR != 3
bool roboticslab::CuiAbsolute::getBemfParamRaw(int j, double *bemf)
{
    CD_INFO("\n");
    return true;
}

bool roboticslab::CuiAbsolute::setBemfParamRaw(int j, double bemf)
{
    CD_INFO("\n");
    return true;
}

bool roboticslab::CuiAbsolute::setTorquePidRaw(int j, const yarp::dev::Pid &pid)
{
    CD_INFO("\n");
    return true;
}

bool roboticslab::CuiAbsolute::setTorquePidsRaw(const yarp::dev::Pid *pids)
{
    CD_ERROR("\n");
    return false;
}

bool roboticslab::CuiAbsolute::setTorqueErrorLimitRaw(int j, double limit)
{
    CD_INFO("\n");
    return true;
}

bool roboticslab::CuiAbsolute::setTorqueErrorLimitsRaw(const double *limits)
{
    CD_ERROR("\n");
    return false;
}

bool roboticslab::CuiAbsolute::getTorqueErrorRaw(int j, double *err)
{
    CD_INFO("\n");
    return true;
}

bool roboticslab::CuiAbsolute::getTorqueErrorsRaw(double *errs)
{
    CD_ERROR("\n");
    return false;
}

bool roboticslab::CuiAbsolute::getTorquePidOutputRaw(int j, double *out)
{
    CD_INFO("\n");
    return true;
}

bool roboticslab::CuiAbsolute::getTorquePidOutputsRaw(double *outs)
{
    CD_ERROR("\n");
    return false;
}

bool roboticslab::CuiAbsolute::getTorquePidRaw(int j, yarp::dev::Pid *pid)
{
    CD_INFO("\n");
    return true;
}

bool roboticslab::CuiAbsolute::getTorquePidsRaw(yarp::dev::Pid *pids)
{
    CD_ERROR("\n");
    return false;
}

bool roboticslab::CuiAbsolute::getTorqueErrorLimitRaw(int j, double *limit)
{
    CD_INFO("\n");
    return true;
}

bool roboticslab::CuiAbsolute::getTorqueErrorLimitsRaw(double *limits)
{
    CD_ERROR("\n");
    return false;
}

bool roboticslab::CuiAbsolute::resetTorquePidRaw(int j)
{
    CD_INFO("\n");
    return true;
}

bool roboticslab::CuiAbsolute::disableTorquePidRaw(int j)
{
    CD_INFO("\n");
    return true;
}

bool roboticslab::CuiAbsolute::enableTorquePidRaw(int j)
{
    CD_INFO("\n");
    return true;
}

bool roboticslab::CuiAbsolute::setTorqueOffsetRaw(int j, double v)
{
    CD_INFO("\n");
    return true;
}
#endif // YARP_VERSION_MAJOR != 3
