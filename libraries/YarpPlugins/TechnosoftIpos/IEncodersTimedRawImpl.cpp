// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "TechnosoftIpos.hpp"

// ------------------ IEncodersTimedRaw Related -----------------------------------------

bool roboticslab::TechnosoftIpos::getEncodersTimedRaw(double *encs, double *time)
{
    CD_ERROR("Missing implementation\n");
    return false;
}

//---------------------------------------------------------------------------------------

bool roboticslab::TechnosoftIpos::getEncoderTimedRaw(int j, double *enc, double *time)
{
    //CD_INFO("(%d)\n", j);  //-- Too verbose in controlboardwrapper2 stream.
    CHECK_JOINT(j);
    *enc =  lastEncoderRead.queryPosition();
    *time = lastEncoderRead.queryTime();
    return true;
}

// -----------------------------------------------------------------------------
