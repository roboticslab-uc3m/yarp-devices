// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "PlaybackThread.hpp"

namespace teo {

// -----------------------------------------------------------------------------

bool PlaybackThread::setIRunnable(IRunnable* iRunnable)
{
    _iRunnable = iRunnable;
}

// -----------------------------------------------------------------------------

bool PlaybackThread::play()
{
    initTime = std::numeric_limits<double>::quiet_NaN();
    setState(PLAYING);
    return true;
}

// -----------------------------------------------------------------------------

bool PlaybackThread::pause()
{
    setState(NOT_PLAYING);
    return true;
}

// -----------------------------------------------------------------------------

bool PlaybackThread::stopPlay()
{
    setState(NOT_PLAYING);
    this->reset();
    return true;
}

// -----------------------------------------------------------------------------

bool PlaybackThread::isPlaying()
{
    return getState() == PLAYING;
}

// -----------------------------------------------------------------------------

bool PlaybackThread::setTimeScale(double timeScale)
{
    this->timeScale = timeScale;
    return true;
}

}  // namespace teo
