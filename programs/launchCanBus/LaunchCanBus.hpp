// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __LAUNCH_CAN_BUS_HPP__
#define __LAUNCH_CAN_BUS_HPP__

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
    ~LaunchCanBus() override
    { close(); }

    bool configure(yarp::os::ResourceFinder &rf) override;
    double getPeriod() override;
    bool updateModule() override;
    bool close() override;

private:
    yarp::dev::PolyDriverList canDevices;
    yarp::dev::PolyDriverList wrapperDevices;
    yarp::dev::PolyDriverList calibratorDevices;
};

} // namespace roboticslab

#endif // __LAUNCH_CAN_BUS_HPP__
