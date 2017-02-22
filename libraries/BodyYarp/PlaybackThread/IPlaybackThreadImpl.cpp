// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "PlaybackThread.hpp"

namespace teo {

// -----------------------------------------------------------------------------

bool PlaybackThread::play()
{
    setState(PLAYING);
    return true;
}

// -----------------------------------------------------------------------------

bool PlaybackThread::stopPlay()
{
    setState(NOT_PLAYING);
    return true;
}

// -----------------------------------------------------------------------------

}  // namespace teo
