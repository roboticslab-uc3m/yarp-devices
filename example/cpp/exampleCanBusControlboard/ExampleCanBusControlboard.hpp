// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __TEST_GRIPPER_BOT__
#define __TEST_GRIPPER_BOT__

#include <yarp/os/RFModule.h>
#include <yarp/os/Module.h>
#include <yarp/os/Network.h>
#include <yarp/os/Port.h>
#include <yarp/os/BufferedPort.h>

#include <yarp/dev/PolyDriver.h>

#include <string>

#include "ColorDebug.hpp"

using namespace yarp::os;
using namespace yarp::dev;

namespace teo
{

/**
 * @ingroup exampleCanBusControlboard
 *
 * @brief Launches 1 left arm DoF + left hand wrapped by controlboardwrapper2.
 * A controlboardwrapper2 may be used through a YARP remote_controlboard or directly through low-level YARP
 * controlboardwrapper2 RPC commands.
 * 
 */
class TestBodyBot : public RFModule {

    protected:
        yarp::dev::PolyDriver robotDevice;

        double getPeriod() {return 3.0;}
        bool updateModule();
//        bool interruptModule();
//        int period;

    public:
        TestBodyBot();
        bool configure(ResourceFinder &rf);
};

}  // namespace teo

#endif

