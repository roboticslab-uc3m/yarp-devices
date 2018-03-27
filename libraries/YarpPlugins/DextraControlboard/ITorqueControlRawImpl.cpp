// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "DextraControlboard.hpp"

// ############################ ITorqueControlRaw Related ############################

bool roboticslab::DextraControlboard::getRefTorquesRaw(double *t)
{
    CD_ERROR("\n");
    return false;
}

// -----------------------------------------------------------------------------------

bool roboticslab::DextraControlboard::getRefTorqueRaw(int j, double *t)
{
    CD_INFO("\n");
    return true;
}

// -----------------------------------------------------------------------------------

bool roboticslab::DextraControlboard::setRefTorquesRaw(const double *t)
{
    CD_ERROR("\n");
    return false;
}

// -----------------------------------------------------------------------------------

bool roboticslab::DextraControlboard::setRefTorqueRaw(int j, double t)
{
    CD_INFO("\n");
    return true;
}

// -----------------------------------------------------------------------------------

bool roboticslab::DextraControlboard::getBemfParamRaw(int j, double *bemf)
{
    CD_INFO("\n");
    return true;
}

// -----------------------------------------------------------------------------------

bool roboticslab::DextraControlboard::setBemfParamRaw(int j, double bemf)
{
    CD_INFO("\n");
    return true;
}

// -----------------------------------------------------------------------------------

bool roboticslab::DextraControlboard::setTorquePidRaw(int j, const yarp::dev::Pid &pid)
{
    CD_INFO("\n");
    return true;
}

// -----------------------------------------------------------------------------------

bool roboticslab::DextraControlboard::getTorqueRaw(int j, double *t)
{
    //CD_INFO("\n");  //-- Too verbose in controlboardwrapper2 stream.
    return true;
}

// -----------------------------------------------------------------------------------

bool roboticslab::DextraControlboard::getTorquesRaw(double *t)
{
    CD_ERROR("\n");
    return false;
}

// -----------------------------------------------------------------------------------

bool roboticslab::DextraControlboard::getTorqueRangeRaw(int j, double *min, double *max)
{
    CD_INFO("\n");
    return true;
}

// -----------------------------------------------------------------------------------

bool roboticslab::DextraControlboard::getTorqueRangesRaw(double *min, double *max)
{
    CD_ERROR("\n");
    return false;
}

// -----------------------------------------------------------------------------------

bool roboticslab::DextraControlboard::setTorquePidsRaw(const yarp::dev::Pid *pids)
{
    CD_ERROR("\n");
    return false;
}

// -----------------------------------------------------------------------------------

bool roboticslab::DextraControlboard::setTorqueErrorLimitRaw(int j, double limit)
{
    CD_INFO("\n");
    return true;
}

// -----------------------------------------------------------------------------------

bool roboticslab::DextraControlboard::setTorqueErrorLimitsRaw(const double *limits)
{
    CD_ERROR("\n");
    return false;
}

// -----------------------------------------------------------------------------------

bool roboticslab::DextraControlboard::getTorqueErrorRaw(int j, double *err)
{
    CD_INFO("\n");
    return true;
}

// -----------------------------------------------------------------------------------

bool roboticslab::DextraControlboard::getTorqueErrorsRaw(double *errs)
{
    CD_ERROR("\n");
    return false;
}

// -----------------------------------------------------------------------------------

bool roboticslab::DextraControlboard::getTorquePidOutputRaw(int j, double *out)
{
    CD_INFO("\n");
    return true;
}

// -----------------------------------------------------------------------------------

bool roboticslab::DextraControlboard::getTorquePidOutputsRaw(double *outs)
{
    CD_ERROR("\n");
    return false;
}

// -----------------------------------------------------------------------------------

bool roboticslab::DextraControlboard::getTorquePidRaw(int j, yarp::dev::Pid *pid)
{
    CD_INFO("\n");
    return true;
}

// -----------------------------------------------------------------------------------

bool roboticslab::DextraControlboard::getTorquePidsRaw(yarp::dev::Pid *pids)
{
    CD_ERROR("\n");
    return false;
}

// -----------------------------------------------------------------------------------

bool roboticslab::DextraControlboard::getTorqueErrorLimitRaw(int j, double *limit)
{
    CD_INFO("\n");
    return true;
}

// -----------------------------------------------------------------------------------

bool roboticslab::DextraControlboard::getTorqueErrorLimitsRaw(double *limits)
{
    CD_ERROR("\n");
    return false;
}

// -----------------------------------------------------------------------------------

bool roboticslab::DextraControlboard::resetTorquePidRaw(int j)
{
    CD_INFO("\n");
    return true;
}

// -----------------------------------------------------------------------------------

bool roboticslab::DextraControlboard::disableTorquePidRaw(int j)
{
    CD_INFO("\n");
    return true;
}

// -----------------------------------------------------------------------------------

bool roboticslab::DextraControlboard::enableTorquePidRaw(int j)
{
    CD_INFO("\n");
    return true;
}

// -----------------------------------------------------------------------------------

bool roboticslab::DextraControlboard::setTorqueOffsetRaw(int j, double v)
{
    CD_INFO("\n");
    return true;
}
