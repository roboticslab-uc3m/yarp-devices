// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/**
 *
 * @ingroup manipulation_modules
 * \defgroup launchManipulation launchManipulation
 *
 * @section launchManipulation_legal Legal
 *
 * Copyright: 2013 (C) Universidad Carlos III de Madrid
 *
 * Author: <a href="http://roboticslab.uc3m.es/roboticslab/persona_publ.php?id_pers=72">Juan G. Victores</a>
 *
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see license/LGPL.TXT
 *
 * @section launchManipulation_install Installation
 *
 * The module is compiled when ENABLE_launchManipulation is activated (default: OFF). For further
 * installation steps refer to <a class="el" href="pages.html">your own system installation guidelines</a>.
 *
 * @section launchManipulation_running Running (assuming correct installation)
 *
 * First we must run a YARP name server if it is not running in our current namespace:
\verbatim
[on terminal 1] yarp server
\endverbatim
 * And then launch the actual module:
\verbatim
[on terminal 2] launchManipulation
\endverbatim
 *
 * @section launchManipulation_modify Modify
 *
 * This file can be edited at
 * $MANIPULATION_ROOT/src/modules/launchManipulation/main.cpp
 *
 */

#include "LaunchLocomotion.hpp"

using namespace yarp::os;
using namespace yarp::dev;

YARP_DECLARE_PLUGINS(BodyYarp);

int main(int argc, char *argv[]) {

    YARP_REGISTER_PLUGINS(BodyYarp);

    ResourceFinder rf;
    rf.setVerbose(true);
    rf.setDefaultContext("launchManipulation/conf");
    rf.setDefaultConfigFile("launchManipulation.ini");
    rf.configure("TEO_ROOT", argc, argv);

    CD_INFO("Checking for yarp network...\n");
    Network yarp;
    if (!yarp.checkNetwork()) {
        CD_ERROR("Found no yarp network (try running \"yarpserver &\"), bye!\n");
        return -1;
    }
    CD_SUCCESS("Found yarp network.\n");

    LaunchLocomotion mod;
    return mod.runModule(rf);
}

