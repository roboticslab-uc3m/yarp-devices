// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/**
 *
 * @ingroup yarp_devices_programs
 * \defgroup canBusLauncher canBusLauncher
 *
 * @brief Creates an instance of roboticslab::CanBusLauncher.
 */

#include <yarp/os/Network.h>
#include <yarp/os/ResourceFinder.h>

#include <ColorDebug.h>

#include "CanBusLauncher.hpp"

int main(int argc, char *argv[])
{
    yarp::os::ResourceFinder rf;
    rf.setVerbose(true);
    rf.setDefaultContext("canBusLauncher");
    rf.setDefaultConfigFile("canBusLauncher.ini");
    rf.configure(argc, argv);

    CD_INFO("Checking for yarp network... ");
    yarp::os::Network yarp;

    if (!yarp.checkNetwork())
    {
        CD_ERROR_NO_HEADER("[fail]\n");
        CD_INFO("Found no yarp network (try running \"yarpserver &\"), bye!\n");
        return 1;
    }

    CD_SUCCESS_NO_HEADER("[ok]\n");

    roboticslab::CanBusLauncher mod;

    if (mod.configure(rf))
    {
        return mod.runModule();
    }
    else
    {
        return mod.close() ? 0 : 1;
    }
}
