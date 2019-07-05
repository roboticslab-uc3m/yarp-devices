// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "DextraControlboardUSB.hpp"

#include <ColorDebug.h>

// ############################ ITorqueControl Related ############################

bool roboticslab::DextraControlboardUSB::getRefTorques(double *t)
{
    CD_ERROR("\n");
    return false;
}

// -----------------------------------------------------------------------------------

bool roboticslab::DextraControlboardUSB::getRefTorque(int j, double *t)
{
    CD_INFO("\n");
    return true;
}

// -----------------------------------------------------------------------------------

bool roboticslab::DextraControlboardUSB::setRefTorques(const double *t)
{
    CD_ERROR("\n");
    return false;
}

// -----------------------------------------------------------------------------------

bool roboticslab::DextraControlboardUSB::setRefTorque(int j, double t)
{
    CD_INFO("\n");
    return true;
}

// -----------------------------------------------------------------------------------

bool roboticslab::DextraControlboardUSB::getTorque(int j, double *t)
{
    //CD_INFO("\n");  //-- Too verbose in controlboardwrapper2 stream.
    return true;
}

// -----------------------------------------------------------------------------------

bool roboticslab::DextraControlboardUSB::getTorques(double *t)
{
    //CD_ERROR("\n");
    return false;
}

// -----------------------------------------------------------------------------------

bool roboticslab::DextraControlboardUSB::getTorqueRange(int j, double *min, double *max)
{
    CD_INFO("\n");
    return true;
}

// -----------------------------------------------------------------------------------

bool roboticslab::DextraControlboardUSB::getTorqueRanges(double *min, double *max)
{
    CD_ERROR("\n");
    return false;
}

// -----------------------------------------------------------------------------------
