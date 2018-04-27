// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "TextilesHand.hpp"

// ############################ ITorqueControlRaw Related ############################

bool roboticslab::TextilesHand::getRefTorquesRaw(double *t)
{
    CD_ERROR("\n");
    return false;
}

// -----------------------------------------------------------------------------------

bool roboticslab::TextilesHand::getRefTorqueRaw(int j, double *t)
{
    CD_INFO("\n");
    return true;
}

// -----------------------------------------------------------------------------------

bool roboticslab::TextilesHand::setRefTorquesRaw(const double *t)
{
    CD_ERROR("\n");
    return false;
}

// -----------------------------------------------------------------------------------

bool roboticslab::TextilesHand::setRefTorqueRaw(int j, double t)
{
    CD_INFO("\n");
    return true;
}

// -----------------------------------------------------------------------------------

bool roboticslab::TextilesHand::getTorqueRaw(int j, double *t)
{
    //CD_INFO("\n");  //-- Too verbose in controlboardwrapper2 stream.
    return true;
}

// -----------------------------------------------------------------------------------

bool roboticslab::TextilesHand::getTorquesRaw(double *t)
{
    CD_ERROR("\n");
    return false;
}

// -----------------------------------------------------------------------------------

bool roboticslab::TextilesHand::getTorqueRangeRaw(int j, double *min, double *max)
{
    CD_INFO("\n");
    return true;
}

// -----------------------------------------------------------------------------------

bool roboticslab::TextilesHand::getTorqueRangesRaw(double *min, double *max)
{
    CD_ERROR("\n");
    return false;
}

// -----------------------------------------------------------------------------------

#if YARP_VERSION_MAJOR != 3
bool roboticslab::TextilesHand::getBemfParamRaw(int j, double *bemf)
{
    CD_INFO("\n");
    return true;
}

// -----------------------------------------------------------------------------------

bool roboticslab::TextilesHand::setBemfParamRaw(int j, double bemf)
{
    CD_INFO("\n");
    return true;
}

// -----------------------------------------------------------------------------------

bool roboticslab::TextilesHand::setTorquePidRaw(int j, const yarp::dev::Pid &pid)
{
    CD_INFO("\n");
    return true;
}

// -----------------------------------------------------------------------------------

bool roboticslab::TextilesHand::setTorquePidsRaw(const yarp::dev::Pid *pids)
{
    CD_ERROR("\n");
    return false;
}

// -----------------------------------------------------------------------------------

bool roboticslab::TextilesHand::setTorqueErrorLimitRaw(int j, double limit)
{
    CD_INFO("\n");
    return true;
}

// -----------------------------------------------------------------------------------

bool roboticslab::TextilesHand::setTorqueErrorLimitsRaw(const double *limits)
{
    CD_ERROR("\n");
    return false;
}

// -----------------------------------------------------------------------------------

bool roboticslab::TextilesHand::getTorqueErrorRaw(int j, double *err)
{
    CD_INFO("\n");
    return true;
}

// -----------------------------------------------------------------------------------

bool roboticslab::TextilesHand::getTorqueErrorsRaw(double *errs)
{
    CD_ERROR("\n");
    return false;
}

// -----------------------------------------------------------------------------------

bool roboticslab::TextilesHand::getTorquePidOutputRaw(int j, double *out)
{
    CD_INFO("\n");
    return true;
}

// -----------------------------------------------------------------------------------

bool roboticslab::TextilesHand::getTorquePidOutputsRaw(double *outs)
{
    CD_ERROR("\n");
    return false;
}

// -----------------------------------------------------------------------------------

bool roboticslab::TextilesHand::getTorquePidRaw(int j, yarp::dev::Pid *pid)
{
    CD_INFO("\n");
    return true;
}

// -----------------------------------------------------------------------------------

bool roboticslab::TextilesHand::getTorquePidsRaw(yarp::dev::Pid *pids)
{
    CD_ERROR("\n");
    return false;
}

// -----------------------------------------------------------------------------------

bool roboticslab::TextilesHand::getTorqueErrorLimitRaw(int j, double *limit)
{
    CD_INFO("\n");
    return true;
}

// -----------------------------------------------------------------------------------

bool roboticslab::TextilesHand::getTorqueErrorLimitsRaw(double *limits)
{
    CD_ERROR("\n");
    return false;
}

// -----------------------------------------------------------------------------------

bool roboticslab::TextilesHand::resetTorquePidRaw(int j)
{
    CD_INFO("\n");
    return true;
}

// -----------------------------------------------------------------------------------

bool roboticslab::TextilesHand::disableTorquePidRaw(int j)
{
    CD_INFO("\n");
    return true;
}

// -----------------------------------------------------------------------------------

bool roboticslab::TextilesHand::enableTorquePidRaw(int j)
{
    CD_INFO("\n");
    return true;
}

// -----------------------------------------------------------------------------------

bool roboticslab::TextilesHand::setTorqueOffsetRaw(int j, double v)
{
    CD_INFO("\n");
    return true;
}
#endif // YARP_VERSION_MAJOR != 3
