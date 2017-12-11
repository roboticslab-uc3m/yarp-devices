// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __TWO_CAN_BUS_THREE_WRAPPERS__
#define __TWO_CAN_BUS_THREE_WRAPPERS__

#include <yarp/os/ResourceFinder.h>
#include <yarp/os/RFModule.h>
#include <yarp/dev/PolyDriver.h>

namespace roboticslab
{

/**
 *
 * @ingroup yarp_devices_libraries
 * \defgroup TwoCanBusThreeWrappers
 * @brief Contains roboticslab::TwoCanBusThreeWrappers.
 */

/**
 * @ingroup TwoCanBusThreeWrappers
 *
 * @brief Launches two CAN bus drivers, and three controlboardwrapper2 instances
 * that wrap corresponding nodes.
 * A controlboardwrapper2 may be used through a YARP remote_controlboard or directly through low-level YARP
 * controlboardwrapper2 RPC commands.
 *
 */
class TwoCanBusThreeWrappers : public yarp::os::RFModule
{

protected:
    yarp::dev::PolyDriver deviceDevCan0;
    yarp::dev::PolyDriver deviceDevCan1;

    yarp::dev::PolyDriver deviceWrapper0;
    yarp::dev::PolyDriver deviceWrapper1;
    yarp::dev::PolyDriver deviceWrapper2;

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
    TwoCanBusThreeWrappers(); // TwoCanBusThreeWrappers();
    bool configure(yarp::os::ResourceFinder &rf);        
};

}  // namespace roboticslab

#endif  // __TWO_CAN_BUS_THREE_WRAPPERS__
