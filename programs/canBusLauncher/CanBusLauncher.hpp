// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __CAN_BUS_LAUNCHER__
#define __CAN_BUS_LAUNCHER__

#include <yarp/os/RFModule.h>
#include <yarp/dev/PolyDriverList.h>

namespace roboticslab
{

/**
 * @ingroup canBusLauncher
 *
 * @brief Launches one or more bus drivers, and controlboardwrapper2 instances
 * that wrap corresponding nodes. A controlboardwrapper2 may be used through a
 * YARP remote_controlboard or directly through low-level YARP controlboardwrapper2
 * RPC commands.
 */
class CanBusLauncher : public yarp::os::RFModule
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

#endif  // __CAN_BUS_LAUNCHER__
