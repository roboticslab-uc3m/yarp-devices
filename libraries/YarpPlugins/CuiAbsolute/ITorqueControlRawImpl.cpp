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
