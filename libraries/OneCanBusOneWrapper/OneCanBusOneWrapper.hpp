// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __ONE_CAN_BUS_THREE_WRAPPER__
#define __ONE_CAN_BUS_THREE_WRAPPER__

#include <yarp/os/ResourceFinder.h>
#include <yarp/os/RFModule.h>

#include <yarp/dev/PolyDriver.h>

namespace roboticslab
{

/**
 *
 * @ingroup yarp_devices_libraries
 * \defgroup OneCanBusOneWrapper
 * @brief Contains roboticslab::OneCanBusOneWrapper.
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
    int timeEncoderWait;
    bool homing;

public:
    OneCanBusOneWrapper();
    bool configure(yarp::os::ResourceFinder &rf);
};

}  // namespace roboticslab

#endif  // __ONE_CAN_BUS_THREE_WRAPPER__
