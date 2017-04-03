// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "TextilesHand.hpp"

// ############################ ITorqueControlRaw Related ############################

bool teo::TextilesHand::setTorqueModeRaw()
{
    CD_ERROR("\n");
    return false;
}

// -----------------------------------------------------------------------------------

bool teo::TextilesHand::getRefTorquesRaw(double *t)
{
    CD_ERROR("\n");
    return false;
}

// -----------------------------------------------------------------------------------

bool teo::TextilesHand::getRefTorqueRaw(int j, double *t)
{
    CD_INFO("\n");
    return true;
}

// -----------------------------------------------------------------------------------

bool teo::TextilesHand::setRefTorquesRaw(const double *t)
{
    CD_ERROR("\n");
    return false;
}

// -----------------------------------------------------------------------------------

bool teo::TextilesHand::setRefTorqueRaw(int j, double t)
{
    CD_INFO("\n");
    return true;
}

// -----------------------------------------------------------------------------------

bool teo::TextilesHand::getBemfParamRaw(int j, double *bemf)
{
    CD_INFO("\n");
    return true;
}

// -----------------------------------------------------------------------------------

bool teo::TextilesHand::setBemfParamRaw(int j, double bemf)
{
    CD_INFO("\n");
    return true;
}

// -----------------------------------------------------------------------------------

bool teo::TextilesHand::setTorquePidRaw(int j, const yarp::dev::Pid &pid)
{
    CD_INFO("\n");
    return true;
}

// -----------------------------------------------------------------------------------

bool teo::TextilesHand::getTorqueRaw(int j, double *t)
{
    //CD_INFO("\n");  //-- Too verbose in controlboardwrapper2 stream.
    return true;
}

// -----------------------------------------------------------------------------------

bool teo::TextilesHand::getTorquesRaw(double *t)
{
    CD_ERROR("\n");
    return false;
}

// -----------------------------------------------------------------------------------

bool teo::TextilesHand::getTorqueRangeRaw(int j, double *min, double *max)
{
    CD_INFO("\n");
    return true;
}

// -----------------------------------------------------------------------------------

bool teo::TextilesHand::getTorqueRangesRaw(double *min, double *max)
{
    CD_ERROR("\n");
    return false;
}

// -----------------------------------------------------------------------------------

bool teo::TextilesHand::setTorquePidsRaw(const yarp::dev::Pid *pids)
{
    CD_ERROR("\n");
    return false;
}

// -----------------------------------------------------------------------------------

bool teo::TextilesHand::setTorqueErrorLimitRaw(int j, double limit)
{
    CD_INFO("\n");
    return true;
}

// -----------------------------------------------------------------------------------

bool teo::TextilesHand::setTorqueErrorLimitsRaw(const double *limits)
{
    CD_ERROR("\n");
    return false;
}

// -----------------------------------------------------------------------------------

bool teo::TextilesHand::getTorqueErrorRaw(int j, double *err)
{
    CD_INFO("\n");
    return true;
}

// -----------------------------------------------------------------------------------

bool teo::TextilesHand::getTorqueErrorsRaw(double *errs)
{
    CD_ERROR("\n");
    return false;
}

// -----------------------------------------------------------------------------------

bool teo::TextilesHand::getTorquePidOutputRaw(int j, double *out)
{
    CD_INFO("\n");
    return true;
}

// -----------------------------------------------------------------------------------

bool teo::TextilesHand::getTorquePidOutputsRaw(double *outs)
{
    CD_ERROR("\n");
    return false;
}

// -----------------------------------------------------------------------------------

bool teo::TextilesHand::getTorquePidRaw(int j, yarp::dev::Pid *pid)
{
    CD_INFO("\n");
    return true;
}

// -----------------------------------------------------------------------------------

bool teo::TextilesHand::getTorquePidsRaw(yarp::dev::Pid *pids)
{
    CD_ERROR("\n");
    return false;
}

// -----------------------------------------------------------------------------------

bool teo::TextilesHand::getTorqueErrorLimitRaw(int j, double *limit)
{
    CD_INFO("\n");
    return true;
}

// -----------------------------------------------------------------------------------

bool teo::TextilesHand::getTorqueErrorLimitsRaw(double *limits)
{
    CD_ERROR("\n");
    return false;
}

// -----------------------------------------------------------------------------------

bool teo::TextilesHand::resetTorquePidRaw(int j)
{
    CD_INFO("\n");
    return true;
}

// -----------------------------------------------------------------------------------

bool teo::TextilesHand::disableTorquePidRaw(int j)
{
    CD_INFO("\n");
    return true;
}

// -----------------------------------------------------------------------------------

bool teo::TextilesHand::enableTorquePidRaw(int j)
{
    CD_INFO("\n");
    return true;
}

// -----------------------------------------------------------------------------------

bool teo::TextilesHand::setTorqueOffsetRaw(int j, double v)
{
    CD_INFO("\n");
    return true;
}
