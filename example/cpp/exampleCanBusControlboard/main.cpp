// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/**
 *
 * @ingroup yarp_devices_programs
 * \defgroup exampleCanBusControlboard exampleCanBusControlboard
 *
 * @brief Creates an instance of teo::ExampleCanBusControlboard.
 *
 * @section exampleCanBusControlboard_legal Legal
 *
 * Copyright: 2013 (C) Universidad Carlos III de Madrid
 *
 * Author: <a href="http://roboticslab.uc3m.es/roboticslab/persona_publ.php?id_pers=72">Juan G. Victores</a>
 *
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see license/LGPL.TXT
 *
 * @section exampleCanBusControlboard_install Installation
 *
 * The module is compiled when ENABLE_exampleCanBusControlboard is activated (default: ON). For further
 * installation steps refer to <a class="el" href="pages.html">your own system installation guidelines</a>.
 *
 * @section exampleCanBusControlboard_running Running (assuming correct installation)
 *
 * First we must make sure a YARP name server is running,
\verbatim
[yarp-devices, terminal 1] yarp detect --write
\endverbatim
 *
 * And run it <b>if it is not running</b> in our current namespace:
\verbatim
[kinematics-dynamics, terminal 1] yarp server
\endverbatim
 *
 * And then launch the actual and specify different options, as in "exampleCanBusControlboard \--option1 value1a value1b \--option2 value2". Specifically, the following options are checked by the CanBusControlboard device (\ref CanBusControlboard class):

\verbatim
device=CanBusControlboard
mode [position]
    position/velocity mode
canDevice [/dev/can0]
    CAN device path
canBitrate [8]
    CAN bitrate
ptModeMs [50]
    PT mode miliseconds
ids
trs
maxs
mins
refAccelerations
refSpeeds
types
\endverbatim
 * Say you have a motoripos device with id 15 and reduction 120, and a motorlacquey device with id 64, both on /dev/can0. The command that enables them and exposes YARP controlboard device ports is:
\verbatim
[yarp-devices, terminal 1] exampleCanBusControlboard --canDevice /dev/can0 --ids 15 64 --types motoripos motorlacquey --trs 120
\endverbatim
 *
 * @section exampleCanBusControlboard_interfacing Interfacing with the exampleCanBusControlboard module
 *
 * The \ref exampleCanBusControlboard module acts as the server part of a network wrapper of the CanBusControlboard class.
 * The implementation maps certain YARP rpc's to CanBusControlboard function calls. Therefore, we can interface
 * with the class from the command-line by typing:
\verbatim
[kinematics-dynamics, terminal 2] yarp rpc /exampleCanBusControlboard/rpc:i
\endverbatim
 * We can send an absolute position joint space movement (say, 5 degrees) to the motoripos such as:
\verbatim
[kinematics-dynamics, terminal 2] set pos 0 5
\endverbatim
 * And should get some kind of feedback, such as:
\verbatim
Response: [ok]
\endverbatim
 * We can send an absolute position movement (say, a position corresponding to a 2000 us PWM) to the motorlacquey such as:
\verbatim
[kinematics-dynamics, terminal 2] set pos 1 2000
\endverbatim
 * And should get some kind of feedback, such as:
\verbatim
Response: [ok]
\endverbatim
 * When finished, Control-C closes programs cleanly. Start by Control-C on kinematics-dynamics' rpc (it's the client part), and then Control-C on yarp-devices' exampleCanBusControlboard (it shuts down the motors correctly).
 *
 * @section exampleCanBusControlboard_modify Modify
 *
 * This file can be edited at
 * yarp-devices/src/modules/exampleCanBusControlboard/main.cpp
 *
 */

#include "ExampleCanBusControlboard.hpp"

using namespace yarp::os;
using namespace yarp::dev;

YARP_DECLARE_PLUGINS(BodyYarp)

int main(int argc, char *argv[])
{

    YARP_REGISTER_PLUGINS(BodyYarp);

    ResourceFinder rf;
    rf.setVerbose(true);
    rf.setDefaultContext("exampleCanBusControlboard");
    rf.setDefaultConfigFile("exampleCanBusControlboard.ini");
    rf.configure(argc, argv);

    CD_INFO("Checking for yarp network...\n");
    Network yarp;
    if (!yarp.checkNetwork())
    {
        CD_ERROR("Found no yarp network (try running \"yarpserver &\"), bye!\n");
        return -1;
    }
    CD_SUCCESS("Found yarp network.\n");

    teo::ExampleCanBusControlboard mod;
    return mod.runModule(rf);
}

