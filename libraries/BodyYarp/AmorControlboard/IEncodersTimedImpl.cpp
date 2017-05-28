// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "AmorControlboard.hpp"

// ------------------ IEncodersTimed related -----------------------------------------

bool roboticslab::AmorControlboard::getEncodersTimed(double *encs, double *time)
{
    //CD_DEBUG("\n");  //-- Way too verbose
    bool ok = true;
    for (unsigned int i = 0; i < AMOR_NUM_JOINTS; i++)
        ok &= getEncoderTimed(i, &(encs[i]), &(time[i]));
    return ok;
}

// -----------------------------------------------------------------------------

bool roboticslab::AmorControlboard::getEncoderTimed(int j, double *encs, double *time)
{
    //CD_DEBUG("(%d)\n", j);  //-- Way too verbose
    if (!indexWithinRange(j))
        return false;

    getEncoder(j, encs);
    *time = yarp::os::Time::now();

    return true;
}

// -----------------------------------------------------------------------------
