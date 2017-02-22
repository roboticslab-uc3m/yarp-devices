// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __JR3__
#define __JR3__

#include <yarp/os/all.h>
#include <yarp/dev/all.h>
#include <yarp/dev/IAnalogSensor.h>
#include <sstream>

#include <fcntl.h>  // ::open
#include <unistd.h>  // ::close

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

        PlaybackThread() {
            initTime = std::numeric_limits<double>::quiet_NaN();
        }

        //  --------- DeviceDriver Declarations. Implementation in DeviceDriverImpl.cpp ---------
        virtual bool open(yarp::os::Searchable& config);
        virtual bool close();

        //  --------- IPlaybackThread Declarations. Implementation in IPlaybackThreadImpl.cpp ---------
        virtual bool play();
        virtual bool stopPlay();

        // --------- RateThread Declarations. Implementation in RateThreadImpl.cpp ---------
        virtual void run();
        virtual void onStop();

    private:
        int timeIdx;
        double initTime;
        double initRow;

        int rowCounter;

        int _state;
        int getState()
        {
            return _state;
        }
        void setState( const int& state)
        {
             _state = state;
             return;
        }
};

}  // namespace teo

#endif  // __JR3__

