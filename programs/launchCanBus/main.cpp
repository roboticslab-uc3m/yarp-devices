// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/**
 * @ingroup yarp_devices_programs
 * @defgroup launchCanBus launchCanBus
 * @brief Creates an instance of roboticslab::LaunchCanBus.
 */

#include <yarp/os/LogStream.h>
#include <yarp/os/Network.h>
#include <yarp/os/ResourceFinder.h>

#include "LaunchCanBus.hpp"

int main(int argc, char * argv[])
{
    yarp::os::ResourceFinder rf;
    rf.setVerbose(true);
    rf.setDefaultContext("launchCanBus");
    rf.setDefaultConfigFile("launchCanBus.ini");
    rf.configure(argc, argv);

    yInfo() << "Checking for yarp network...";
    yarp::os::Network yarp;

    if (!yarp::os::Network::checkNetwork())
    {
        yError() << "Found no yarp network (try running \"yarpserver &\"), bye!";
        return 1;
    }

    roboticslab::LaunchCanBus mod;
    return mod.runModule(rf);
}
