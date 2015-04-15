// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __LAUNCH_MANIPULATION__
#define __LAUNCH_MANIPULATION__

#include <yarp/os/RFModule.h>
#include <yarp/os/Module.h>
#include <yarp/os/Network.h>
#include <yarp/os/Port.h>
#include <yarp/os/BufferedPort.h>

#include <yarp/dev/PolyDriver.h>

#include <string>
#include <stdlib.h>

#include "ColorDebug.hpp"

#define DEFAULT_ROBOT_NAME "/teo"

using namespace yarp::os;
using namespace yarp::dev;

namespace teo
{

/**
 * @ingroup launchManipulation
 *
 * @brief Launches left arm wrapped by controlboardwrapper2 and right arm wrapped by controlboardwrapper2.
 * A controlboardwrapper2 may be used through a YARP remote_controlboard or directly through low-level YARP
 * controlboardwrapper2 RPC commands.
 * 
 */
class LaunchManipulation : public RFModule {

    protected:
        yarp::dev::PolyDriver leftArmDevice;
        yarp::dev::PolyDriver rightArmDevice;

        virtual double getPeriod() {return 3.0;}
        virtual bool updateModule();
        virtual bool close();
//        virtual bool interruptModule();
//        virtual int period;

    public:
        LaunchManipulation();
        bool configure(ResourceFinder &rf);
};

}  // namespace teo

#endif  // __LAUNCH_MANIPULATION__

