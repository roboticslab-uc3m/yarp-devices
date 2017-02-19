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

#include "IPlaybackThread.h"

#define DEFAULT_RATE_MS 20.0
#define DEFAULT_NUM_CHANNELS 6

namespace teo
{

/**
 * @ingroup BodyYarp
 * \defgroup PlaybackThread
 * @brief Contains teo::PlaybackThread.
 */

 /**
 * @ingroup PlaybackThread
 * @brief Implementation for the JR3 sensor. Launch as in: yarpdev --device PlaybackThread --period 20 --name /jr3:o
 *
 */
class PlaybackThread : public yarp::dev::DeviceDriver, public yarp::os::RateThread, IPlaybackThread
{

    public:

        PlaybackThread() : RateThread(DEFAULT_RATE_MS) {
        }

        //  --------- DeviceDriver Declarations. Implementation in DeviceDriverImpl.cpp ---------
        virtual bool open(yarp::os::Searchable& config);
        virtual bool close();

        virtual bool play() { return true; }
    // --------- RateThread Declarations. Implementation in RateThreadImpl.cpp ---------
        virtual void run();

    private:

};

}  // namespace teo

#endif  // __JR3__

