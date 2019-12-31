// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __LAUNCH_CAN_BUS__
#define __LAUNCH_CAN_BUS__

#include <yarp/os/RFModule.h>
#include <yarp/dev/PolyDriverList.h>

namespace roboticslab
{

/**
 * @ingroup launchCanBus
 * @brief Launches one or more CAN bus drivers and controlboardwrapper2 network wrappers.
 */
class LaunchCanBus : public yarp::os::RFModule
{
public:
    bool configure(yarp::os::ResourceFinder &rf);
    virtual double getPeriod();
    virtual bool updateModule();
    virtual bool close();

private:
    yarp::dev::PolyDriverList canDevices;
    yarp::dev::PolyDriverList wrapperDevices;
    yarp::dev::PolyDriverList calibratorDevices;
};

}  // namespace roboticslab

#endif  // __LAUNCH_CAN_BUS__
