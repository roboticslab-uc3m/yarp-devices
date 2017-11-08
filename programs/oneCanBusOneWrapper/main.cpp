// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/**
 *
 * @ingroup yarp_devices_programs
 * \defgroup oneCanBusOneWrapper oneCanBusOneWrapper
 *
 * @brief Creates an instance of roboticslab::OneCanBusOneWrapper.
 *
 * @section oneCanBusOneWrapper_legal Legal
 *
 * Copyright: 2013 (C) Universidad Carlos III de Madrid
 *
 * Author: <a href="http://roboticslab.uc3m.es/roboticslab/persona_publ.php?id_pers=72">Juan G. Victores</a>
 *
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see license/LGPL.TXT
 *
 * @section oneCanBusOneWrapper_install Installation
 *
 * The module is compiled when ENABLE_oneCanBusOneWrapper is activated (default: ON). For further
 * installation steps refer to <a class="el" href="pages.html">your own system installation guidelines</a>.
 *
 * @section oneCanBusOneWrapper_running Running (assuming correct installation)
 *
 * First we must run a YARP name server if it is not running in our current namespace:
\verbatim
[on terminal 1] yarp server
\endverbatim
 * And then launch the actual module:
\verbatim
[on terminal 2] oneCanBusOneWrapper
\endverbatim
 *
 * @section oneCanBusOneWrapper_modify Modify
 *
 * This file can be edited at
 * programs/oneCanBusOneWrapper/main.cpp
 *
 */

#include "OneCanBusOneWrapper.hpp"


//YARP_DECLARE_PLUGINS(YarpPlugins)

int main(int argc, char *argv[])
{

    //YARP_REGISTER_PLUGINS(YarpPlugins);

    yarp::os::ResourceFinder rf;
    rf.setVerbose(true);
    rf.setDefaultContext("oneCanBusOneWrapper");
    rf.setDefaultConfigFile("oneCanBusOneWrapper.ini");
    rf.configure(argc, argv);

    CD_INFO("Checking for yarp network...\n");
    yarp::os::Network yarp;
    if (!yarp.checkNetwork())
    {
        CD_ERROR("Found no yarp network (try running \"yarpserver &\"), bye!\n");
        return 1;
    }
    CD_SUCCESS("Found yarp network.\n");

    roboticslab::OneCanBusOneWrapper mod;
    return mod.runModule(rf);
}

