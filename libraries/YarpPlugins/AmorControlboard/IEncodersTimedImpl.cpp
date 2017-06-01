// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "AmorControlboard.hpp"

#include <algorithm>

// ------------------ IEncodersTimed related -----------------------------------------

bool roboticslab::AmorControlboard::getEncodersTimed(double *encs, double *time)
{
    CD_DEBUG("\n");
    double now = yarp::os::Time::now();
    std::fill_n(time, AMOR_NUM_JOINTS, now);
    return getEncoders(encs);
}

// -----------------------------------------------------------------------------

bool roboticslab::AmorControlboard::getEncoderTimed(int j, double *encs, double *time)
{
    //CD_DEBUG("(%d)\n", j);
    *time = yarp::os::Time::now();
    return getEncoder(j, encs);
}

// -----------------------------------------------------------------------------
