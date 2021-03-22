// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/**
 * @ingroup yarp_devices_programs
 * @defgroup dumpCanBus dumpCanBus
 * @brief Creates an instance of roboticslab::DumpCanBus.
 *
 * This app connects to a remote /dump:o port that streams CAN frames flowing
 * through a physical bus. Messages are print in a human-friendly format. To
 * override preset CANopen function codes and print bare node IDs, pass the
 * <code>--no-can-open</code> option.
 */

#include <yarp/os/LogStream.h>
#include <yarp/os/Network.h>
#include <yarp/os/ResourceFinder.h>

#include "DumpCanBus.hpp"

int main(int argc, char *argv[])
{
    yarp::os::ResourceFinder rf;
    rf.setVerbose(true);
    rf.setDefaultContext("dumpCanBus");
    rf.setDefaultConfigFile("dumpCanBus.ini");
    rf.configure(argc, argv);

    yarp::os::Network yarp;
    yInfo() << "Checking for yarp network...";

    if (!yarp::os::Network::checkNetwork())
    {
        yError() << "YARP network not found";
        return 1;
    }

    roboticslab::DumpCanBus mod;
    return mod.runModule(rf);
}
