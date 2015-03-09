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
 * The LaunchManipulation class launches the manipulators as controlboards.
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

