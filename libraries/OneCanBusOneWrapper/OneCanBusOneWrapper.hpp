// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __TWO_CAN_BUS_THREE_WRAPPERS__
#define __TWO_CAN_BUS_THREE_WRAPPERS__

#include <yarp/os/RFModule.h>
#include <yarp/os/Module.h>
#include <yarp/os/Network.h>
#include <yarp/os/Port.h>
#include <yarp/os/BufferedPort.h>

#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/Wrapper.h>

#include <string>
#include <stdlib.h>

#include "ColorDebug.hpp"


namespace teo
{

/**
 *
 * @ingroup teo_body_libraries
 * \defgroup OneCanBusOneWrapper
 * @brief Contains teo::OneCanBusOneWrapper.
 */

/**
 * @ingroup OneCanBusOneWrapper
 *
 * @brief Launches one CAN bus drivers, and one controlboardwrapper2 instance
 * that wraps the corresponding nodes.
 * A controlboardwrapper2 may be used through a YARP remote_controlboard or directly through low-level YARP
 * controlboardwrapper2 RPC commands.
 *
 */
class OneCanBusOneWrapper : public yarp::os::RFModule
{

protected:
    yarp::dev::PolyDriver deviceDevCan0;

    yarp::dev::PolyDriver deviceWrapper0;

    virtual double getPeriod()
    {
        return 3.0;
    }
    virtual bool updateModule();
    virtual bool close();
//        virtual bool interruptModule();
//        virtual int period;

public:
    OneCanBusOneWrapper();
    bool configure(yarp::os::ResourceFinder &rf);
};

}  // namespace teo

#endif  // __TWO_CAN_BUS_THREE_WRAPPERS__

