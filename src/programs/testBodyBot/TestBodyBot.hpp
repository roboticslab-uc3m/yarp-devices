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

#define DEFAULT_ROBOT_NAME "/teo"

using namespace yarp::os;
using namespace yarp::dev;

/**
 * @ingroup testBodyBot
 *
 * The TestBodyBot class tests the SkymegaBot class as a controlboard.
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

#endif

