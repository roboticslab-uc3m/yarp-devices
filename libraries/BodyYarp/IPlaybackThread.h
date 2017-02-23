// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __I_PLAYBACK_THREAD__
#define __I_PLAYBACK_THREAD__

#include <vector>

#include "IRunnable.h"

namespace teo
{

/**
 *
 * @brief Abstract base for PlaybackThread.
 *
 */
class IPlaybackThread
{
public:
    /**
     * Destructor.
     */
    virtual ~IPlaybackThread() {}

    virtual bool play() = 0;
    virtual bool pause() = 0;
    virtual bool stopPlay() = 0;
    virtual bool isPlaying() = 0;
    virtual bool setTimeScale(double timeScale) = 0;
    virtual bool setIRunnable(IRunnable* iRunnable) = 0;
};

}  // namespace teo

#endif  //  __I_PLAYBACK_THREAD__
