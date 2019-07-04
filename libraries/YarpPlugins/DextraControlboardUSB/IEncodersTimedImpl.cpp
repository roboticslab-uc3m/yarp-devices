// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "DextraControlboardUSB.hpp"

#include <yarp/os/Time.h>

#include <ColorDebug.h>

// ------------------ IEncodersTimed Related -----------------------------------------

bool roboticslab::DextraControlboardUSB::getEncodersTimed(double *encs, double *time)
{
    CD_DEBUG("\n");
    *time = yarp::os::Time::now();
    return getEncoders(encs);
}

// -----------------------------------------------------------------------------

bool roboticslab::DextraControlboardUSB::getEncoderTimed(int j, double *enc, double *time)
{
    //CD_DEBUG("(%d)\n", j);  //-- Too verbose in controlboardwrapper2 stream.
    CHECK_JOINT(j);
    *time = yarp::os::Time::now();
    return getEncoder(j, enc);
}

// -----------------------------------------------------------------------------
