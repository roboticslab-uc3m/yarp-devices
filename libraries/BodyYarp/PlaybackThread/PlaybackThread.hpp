// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __PLAYBACK_THREAD__
#define __PLAYBACK_THREAD__

#include <yarp/os/all.h>
#include <yarp/dev/all.h>
#include <sstream>

#include <limits>  // NAN

//#define CD_FULL_FILE  //-- Can be globally managed from father CMake. Good for debugging with polymorphism.
//#define CD_HIDE_DEBUG  //-- Can be globally managed from father CMake.
//#define CD_HIDE_SUCCESS  //-- Can be globally managed from father CMake.
//#define CD_HIDE_INFO  //-- Can be globally managed from father CMake.
//#define CD_HIDE_WARNING  //-- Can be globally managed from father CMake.
//#define CD_HIDE_ERROR  //-- Can be globally managed from father CMake.
#include "ColorDebug.hpp"

#include "Playback.hpp"

#include "IPlaybackThread.h"

#define NOT_PLAYING 0
#define PLAYING 1

#define DEFAULT_FILE_NAME "test.txt"
#define DEFAULT_TIME_IDX 0
#define DEFAULT_TIME_SCALE 1

namespace teo
{

/**
 * @ingroup BodyYarp
 * \defgroup PlaybackThread
 * @brief Contains teo::PlaybackThread.
 */

 /**
 * @ingroup PlaybackThread
 * @brief Implementation for the PlaybackThread.
 *
 */
class PlaybackThread : public yarp::dev::DeviceDriver, public IPlaybackThread, public yarp::os::Thread, public Playback
{

    public:

        PlaybackThread() { }

        //  --------- DeviceDriver Declarations. Implementation in DeviceDriverImpl.cpp ---------
        virtual bool open(yarp::os::Searchable& config);
        virtual bool close();

        //  --------- IPlaybackThread Declarations. Implementation in IPlaybackThreadImpl.cpp ---------
        virtual bool play();
        virtual bool pause();
        virtual bool stopPlay();
        virtual bool isPlaying();
        virtual bool setTimeScale(double timeScale);
        virtual bool setIRunnable(IRunnable* iRunnable);

        // --------- RateThread Declarations. Implementation in RateThreadImpl.cpp ---------
        virtual void run();
        virtual void onStop();

    private:
        int timeIdx;
        double timeScale;
        double initTime;
        double initRow;

        yarp::os::Bottle mask;

        IRunnable* _iRunnable;

        int _state;
        yarp::os::Semaphore _stateSemaphore;
        int getState()
        {
            _stateSemaphore.wait();
            int tmp = _state;
            _stateSemaphore.post();
            return tmp;
        }
        void setState( const int& state)
        {
            //CD_DEBUG("%d\n",state);
            _stateSemaphore.wait();
            _state = state;
            _stateSemaphore.post();
            return;
        }
};

}  // namespace teo

#endif  // __PLAYBACK_THREAD__

