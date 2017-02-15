// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __PLAYBACK__
#define __PLAYBACK__

#include <yarp/os/all.h>
#include <yarp/dev/all.h>
#include <sstream>

#include <fstream>

//#define CD_FULL_FILE  //-- Can be globally managed from father CMake. Good for debugging with polymorphism.
//#define CD_HIDE_DEBUG  //-- Can be globally managed from father CMake.
//#define CD_HIDE_SUCCESS  //-- Can be globally managed from father CMake.
//#define CD_HIDE_INFO  //-- Can be globally managed from father CMake.
//#define CD_HIDE_WARNING  //-- Can be globally managed from father CMake.
//#define CD_HIDE_ERROR  //-- Can be globally managed from father CMake.
#include "ColorDebug.hpp"

#define DEFAULT_RATE_MS 20.0
#define DEFAULT_NUM_CHANNELS 6

namespace teo
{

/**
 * @ingroup BodyYarp
 * \defgroup Playback
 * @brief Contains teo::Playback.
 */

 /**
 * @ingroup Playback
 * @brief Implementation for the PLAYBACK sensor. Launch as in: yarpdev --device Playback --period 20 --name /jr3:o
 *
 */
class Playback : public yarp::dev::DeviceDriver
{

    public:

        Playback() {
        }

        //  --------- DeviceDriver Declarations. Implementation in DeviceDriverImpl.cpp ---------
        virtual bool open(yarp::os::Searchable& config);
        virtual bool close();

    private:
        std::ifstream file;

};

}  // namespace teo

#endif  // __PLAYBACK__

