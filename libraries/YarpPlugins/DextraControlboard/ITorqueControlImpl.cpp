// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "DextraControlboard.hpp"

// ############################ ITorqueControl Related ############################

bool roboticslab::DextraControlboard::getRefTorques(double *t)
{
    CD_ERROR("\n");
    return false;
}

// -----------------------------------------------------------------------------------

bool roboticslab::DextraControlboard::getRefTorque(int j, double *t)
{
    CD_INFO("\n");
    return true;
}

// -----------------------------------------------------------------------------------

bool roboticslab::DextraControlboard::setRefTorques(const double *t)
{
    CD_ERROR("\n");
    return false;
}

// -----------------------------------------------------------------------------------

bool roboticslab::DextraControlboard::setRefTorque(int j, double t)
{
    CD_INFO("\n");
    return true;
}

// -----------------------------------------------------------------------------------

bool roboticslab::DextraControlboard::getBemfParam(int j, double *bemf)
{
    CD_INFO("\n");
    return true;
}

// -----------------------------------------------------------------------------------

bool roboticslab::DextraControlboard::setBemfParam(int j, double bemf)
{
    CD_INFO("\n");
    return true;
}

// -----------------------------------------------------------------------------------

bool roboticslab::DextraControlboard::setTorquePid(int j, const yarp::dev::Pid &pid)
{
    CD_INFO("\n");
    return true;
}

// -----------------------------------------------------------------------------------

bool roboticslab::DextraControlboard::getTorque(int j, double *t)
{
    //CD_INFO("\n");  //-- Too verbose in controlboardwrapper2 stream.
    return true;
}

// -----------------------------------------------------------------------------------

bool roboticslab::DextraControlboard::getTorques(double *t)
{
    CD_ERROR("\n");
    return false;
}

// -----------------------------------------------------------------------------------

bool roboticslab::DextraControlboard::getTorqueRange(int j, double *min, double *max)
{
    CD_INFO("\n");
    return true;
}

// -----------------------------------------------------------------------------------

bool roboticslab::DextraControlboard::getTorqueRanges(double *min, double *max)
{
    CD_ERROR("\n");
    return false;
}

// -----------------------------------------------------------------------------------

bool roboticslab::DextraControlboard::setTorquePids(const yarp::dev::Pid *pids)
{
    CD_ERROR("\n");
    return false;
}

// -----------------------------------------------------------------------------------

bool roboticslab::DextraControlboard::setTorqueErrorLimit(int j, double limit)
{
    CD_INFO("\n");
    return true;
}

// -----------------------------------------------------------------------------------

bool roboticslab::DextraControlboard::setTorqueErrorLimits(const double *limits)
{
    CD_ERROR("\n");
    return false;
}

// -----------------------------------------------------------------------------------

bool roboticslab::DextraControlboard::getTorqueError(int j, double *err)
{
    CD_INFO("\n");
    return true;
}

// -----------------------------------------------------------------------------------

bool roboticslab::DextraControlboard::getTorqueErrors(double *errs)
{
    CD_ERROR("\n");
    return false;
}

// -----------------------------------------------------------------------------------

bool roboticslab::DextraControlboard::getTorquePidOutput(int j, double *out)
{
    CD_INFO("\n");
    return true;
}

// -----------------------------------------------------------------------------------

bool roboticslab::DextraControlboard::getTorquePidOutputs(double *outs)
{
    CD_ERROR("\n");
    return false;
}

// -----------------------------------------------------------------------------------

bool roboticslab::DextraControlboard::getTorquePid(int j, yarp::dev::Pid *pid)
{
    CD_INFO("\n");
    return true;
}

// -----------------------------------------------------------------------------------

bool roboticslab::DextraControlboard::getTorquePids(yarp::dev::Pid *pids)
{
    CD_ERROR("\n");
    return false;
}

// -----------------------------------------------------------------------------------

bool roboticslab::DextraControlboard::getTorqueErrorLimit(int j, double *limit)
{
    CD_INFO("\n");
    return true;
}

// -----------------------------------------------------------------------------------

bool roboticslab::DextraControlboard::getTorqueErrorLimits(double *limits)
{
    CD_ERROR("\n");
    return false;
}

// -----------------------------------------------------------------------------------

bool roboticslab::DextraControlboard::resetTorquePid(int j)
{
    CD_INFO("\n");
    return true;
}

// -----------------------------------------------------------------------------------

bool roboticslab::DextraControlboard::disableTorquePid(int j)
{
    CD_INFO("\n");
    return true;
}

// -----------------------------------------------------------------------------------

bool roboticslab::DextraControlboard::enableTorquePid(int j)
{
    CD_INFO("\n");
    return true;
}

// -----------------------------------------------------------------------------------

bool roboticslab::DextraControlboard::setTorqueOffset(int j, double v)
{
    CD_INFO("\n");
    return true;
}
