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
#endif // YARP_VERSION_MAJOR != 3
