// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "Playback.hpp"

// -----------------------------------------------------------------------------

bool teo::Playback::open(yarp::os::Searchable& config)
{


    return true;
}

// -----------------------------------------------------------------------------

bool teo::Playback::close()
{
    ::close(fd);
    return true;
}

// -----------------------------------------------------------------------------
