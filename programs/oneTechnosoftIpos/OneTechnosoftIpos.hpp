// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __LAUNCH_MANIPULATION__
#define __LAUNCH_MANIPULATION__

#include <yarp/os/RFModule.h>
#include <yarp/os/Module.h>
#include <yarp/os/Network.h>
#include <yarp/os/Port.h>
#include <yarp/os/BufferedPort.h>

#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/Wrapper.h>

#include <string>
#include <stdlib.h>

#include "ICanBusSharer.h"
#include "ColorDebug.hpp"

using namespace yarp::os;

namespace teo
{

/**
 * @ingroup oneTechnosoftIpos
 *
 * @brief Launches two CAN bus drivers, and three controlboardwrapper2 instances
 * that wrap corresponding nodes for: /teo/leftArm, /teo/rightArm, /teo/head.
 * A controlboardwrapper2 may be used through a YARP remote_controlboard or directly through low-level YARP
 * controlboardwrapper2 RPC commands.
 * 
 */
class OneTechnosoftIpos : public RFModule {

    public:
        OneTechnosoftIpos();
        bool configure(ResourceFinder &rf);

    protected:

        virtual double getPeriod() {return 3.0;}
        virtual bool updateModule();
        virtual bool close();
    //        virtual bool interruptModule();
    //        virtual int period;

        /** A CAN device. */
        yarp::dev::PolyDriver canBusDevice;
        CanBusHico* iCanBus;


        yarp::dev::PolyDriver dd;
};

}  // namespace teo

#endif  // __LAUNCH_MANIPULATION__

