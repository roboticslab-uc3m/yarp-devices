// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/**
 *
 * @ingroup manipulation_modules
 * \defgroup testBodyBot testBodyBot
 *
 * Creates an instance of GripperBot wrapped in a YARP controlboard. This means that it may be used
 * through a YARP remote_controlboard or directly through low-level YARP controlboard RPC commands.
 *
 * @section testBodyBot_legal Legal
 *
 * Copyright: 2013 (C) Universidad Carlos III de Madrid
 *
 * Author: <a href="http://roboticslab.uc3m.es/roboticslab/persona_publ.php?id_pers=72">Juan G. Victores</a>
 *
 * Contrib: Iván (author oth the canOpen module)
 *
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see license/LGPL.TXT
 *
 * @section testBodyBot_install Installation
 *
 * The module is compiled when ENABLE_testBodyBot is activated (default: OFF). For further
 * installation steps refer to <a class="el" href="pages.html">your own system installation guidelines</a>.
 *
 * @section testBodyBot_running Running (assuming correct installation)
 *
 * First we must run a YARP name server if it is not running in our current namespace:
\verbatim
[on terminal 1] yarp server
\endverbatim
 * And then launch the actual module:
\verbatim
[on terminal 2] testBodyBot
\endverbatim
 *
 * @section testBodyBot_interfacing Interfacing with the testBodyBot module
 *
 * The \ref testBodyBot module acts as the server part of a network wrapper of the GripperBot class.
 * The implementation maps certain YARP rpc's to tGripperBot function calls. Therefore, we can interface
 * with the class from the command-line by typing:
\verbatim
[on terminal 3] yarp rpc /gripperbot/rpc:i
\endverbatim
 * We can send an absolute position joint space movement such as:
\verbatim
[on terminal 3] set pos 0 2000
\endverbatim
 * And should get some kind of feedback, such as:
\verbatim
Response: [ok]
\endverbatim
 *
 * @section testBodyBot_modify Modify
 *
 * This file can be edited at
 * $MANIPULATION_ROOT/src/modules/testBodyBot/main.cpp
 *
 */

#include "TestBodyBot.hpp"

using namespace yarp::os;
using namespace yarp::dev;

YARP_DECLARE_PLUGINS(BodyYarp);

int main(int argc, char *argv[]) {

    YARP_REGISTER_PLUGINS(BodyYarp);

    ResourceFinder rf;
    rf.setVerbose(true);
    rf.setDefaultContext("testBodyBot/conf");
    rf.setDefaultConfigFile("testBodyBot.ini");
    rf.configure("TEO_ROOT", argc, argv);

    CD_INFO("Checking for yarp network...\n");
    Network yarp;
    if (!yarp.checkNetwork()) {
        CD_ERROR("Found no yarp network (try running \"yarpserver &\"), bye!\n");
        return -1;
    }
    CD_SUCCESS("Found yarp network.\n");

    TestBodyBot mod;
    return mod.runModule(rf);
}

