// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "PlaybackThread.hpp"

// -----------------------------------------------------------------------------

bool teo::PlaybackThread::open(yarp::os::Searchable& config)
{
    CD_SUCCESS("\n");
    return true;
}

// -----------------------------------------------------------------------------

bool teo::PlaybackThread::close()
{
    return true;
}

// -----------------------------------------------------------------------------
