// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __LAUNCH_LOCOMOTION__
#define __LAUNCH_LOCOMOTION__

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
 * @ingroup launchLocomotion
 *
 * @brief The LaunchLocomotion class launches the legs as controlboards.
 * 
 */
class LaunchLocomotion : public RFModule {

    protected:
        yarp::dev::PolyDriver leftLegDevice;
        yarp::dev::PolyDriver rightLegDevice;

        virtual double getPeriod() {return 3.0;}
        virtual bool updateModule();
        virtual bool close();
//        virtual bool interruptModule();
//        virtual int period;

    public:
        LaunchLocomotion();
        bool configure(ResourceFinder &rf);
};

}  // namespace teo

#endif  // __LAUNCH_LOCOMOTION__

