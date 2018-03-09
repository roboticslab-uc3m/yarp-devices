// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "FakeControlboard.hpp"

#include <yarp/os/Time.h>

// ------------------ IEncodersTimed Related -----------------------------------------

bool roboticslab::FakeControlboard::getEncodersTimed(double *encs, double *time)
{
    //CD_DEBUG("\n");  //-- Way too verbose
    bool ok = true;

    for (unsigned int i = 0; i < axes; i++)
    {
        ok &= getEncoderTimed(i, &encs[i], &time[i]);
    }

    return ok;
}

// -----------------------------------------------------------------------------

bool roboticslab::FakeControlboard::getEncoderTimed(int j, double *encs, double *time)
{
    //CD_DEBUG("(%d)\n",j);  //-- Way too verbose

    getEncoder(j, encs);
    *time = yarp::os::Time::now();

    return true;
}

// -----------------------------------------------------------------------------
