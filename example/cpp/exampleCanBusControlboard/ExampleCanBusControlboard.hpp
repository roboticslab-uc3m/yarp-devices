// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __EXAMPLE_CAN_BUS_CONTROLBOARD__
#define __EXAMPLE_CAN_BUS_CONTROLBOARD__

#include <yarp/os/RFModule.h>
#include <yarp/os/Module.h>
#include <yarp/os/Network.h>
#include <yarp/os/Port.h>
#include <yarp/os/BufferedPort.h>

#include <yarp/dev/PolyDriver.h>

#include <string>

#include "ColorDebug.hpp"

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
class ExampleCanBusControlboard : public yarp::os::RFModule
{

protected:
    yarp::dev::PolyDriver robotDevice;

    double getPeriod()
    {
        return 3.0;
    }
    bool updateModule();
//        bool interruptModule();
//        int period;

public:
    ExampleCanBusControlboard();
    bool configure(yarp::os::ResourceFinder &rf);
};

}  // namespace teo

#endif  // __EXAMPLE_CAN_BUS_CONTROLBOARD__
