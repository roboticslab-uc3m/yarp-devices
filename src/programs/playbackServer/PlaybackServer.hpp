// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __PLAYBACK_MANIPULATION__
#define __PLAYBACK_MANIPULATION__

#include <yarp/os/RFModule.h>
#include <yarp/os/Module.h>
#include <yarp/os/Network.h>
#include <yarp/os/Port.h>
#include <yarp/os/BufferedPort.h>

#include <yarp/dev/PolyDriver.h>

#include <string>

#include "ColorDebug.hpp"

#include "PlaybackThread.hpp"
#include "WebResponder.hpp"

#define DEFAULT_WEB_IP "localhost"
#define DEFAULT_WEB_PORT 8080
#define DEFAULT_WEB_NAME "/playbackServer"

#define DEFAULT_FILE_PATH "/home/teo"
#define DEFAULT_FILE_EXTENSION ".txt"

#define DEFAULT_PT_MODE_MS 50

namespace teo
{

/**
 * @ingroup playbackServer
 *
 * @brief Plays back a robot file, offering a web interface.
 * 
 */
class PlaybackServer : public yarp::os::RFModule {

    public:
        PlaybackServer();
        bool configure(yarp::os::ResourceFinder &rf);

    protected:
        yarp::dev::PolyDriver leftArmDevice;
        yarp::dev::PolyDriver rightArmDevice;

        virtual double getPeriod() {return 3.0;}
        virtual bool updateModule();
        virtual bool close();
    //        virtual bool interruptModule();
    //        virtual int period;

        PlaybackThread playbackThread;

        yarp::os::Port server;
        WebResponder responder;

};

}  // namespace teo

#endif  // __PLAYBACK_MANIPULATION__

