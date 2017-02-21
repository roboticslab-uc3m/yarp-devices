// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __JR3__
#define __JR3__

#include <yarp/os/all.h>
#include <yarp/dev/all.h>
#include <yarp/dev/IAnalogSensor.h>
#include <sstream>

#include <fcntl.h>  // ::open
#include <unistd.h>  // ::close

//#define CD_FULL_FILE  //-- Can be globally managed from father CMake. Good for debugging with polymorphism.
//#define CD_HIDE_DEBUG  //-- Can be globally managed from father CMake.
//#define CD_HIDE_SUCCESS  //-- Can be globally managed from father CMake.
//#define CD_HIDE_INFO  //-- Can be globally managed from father CMake.
//#define CD_HIDE_WARNING  //-- Can be globally managed from father CMake.
//#define CD_HIDE_ERROR  //-- Can be globally managed from father CMake.
#include "ColorDebug.hpp"

#include "Playback.hpp"

#include "IPlaybackThread.h"

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
        }

        //  --------- DeviceDriver Declarations. Implementation in DeviceDriverImpl.cpp ---------
        virtual bool open(yarp::os::Searchable& config);
        virtual bool close();

        //  --------- IPlaybackThread Declarations. Implementation in IPlaybackThreadImpl.cpp ---------
        virtual bool play();

        // --------- RateThread Declarations. Implementation in RateThreadImpl.cpp ---------
        virtual void run();

    private:
        int timeIdx;
};

}  // namespace teo

#endif  // __JR3__

