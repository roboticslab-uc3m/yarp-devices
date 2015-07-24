// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/**
 *
 * @ingroup teo_body_programs
 * \defgroup playbackLocomotion playbackLocomotion
 *
 * @brief Creates an instance of teo::PlaybackLocomotion.
 *
 * @section playbackLocomotion_legal Legal
 *
 * Copyright: 2013 (C) Universidad Carlos III de Madrid
 *
 * Author: <a href="http://roboticslab.uc3m.es/roboticslab/persona_publ.php?id_pers=72">Juan G. Victores</a>
 *
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see license/LGPL.TXT
 *
 * @section playbackLocomotion_install Installation
 *
 * The module is compiled when ENABLE_playbackLocomotion is activated (default: OFF). For further
 * installation steps refer to <a class="el" href="pages.html">your own system installation guidelines</a>.
 *
 * @section playbackLocomotion_running Running (assuming correct installation)
 *
 * First we must run a YARP name server if it is not running in our current namespace:
\verbatim
[on terminal 1] yarp server
\endverbatim
 * And then launch the actual module:
\verbatim
[on terminal 2] playbackLocomotion
\endverbatim
 *
 * @section playbackLocomotion_modify Modify
 *
 * This file can be edited at
 * teo-body/src/modules/playbackLocomotion/main.cpp
 *
 */

#include "PlaybackLocomotion.hpp"

using namespace yarp::os;
using namespace yarp::dev;

YARP_DECLARE_PLUGINS(BodyYarp)

int main(int argc, char *argv[]) {

    YARP_REGISTER_PLUGINS(BodyYarp);

    ResourceFinder rf;
    rf.setVerbose(true);
    rf.setDefaultContext("playbackLocomotion");
    rf.setDefaultConfigFile("playbackLocomotion.ini");
    rf.configure(argc, argv);

    CD_INFO("Checking for yarp network...\n");
    Network yarp;
    if (!yarp.checkNetwork()) {
        CD_ERROR("Found no yarp network (try running \"yarpserver &\"), bye!\n");
        return 1;
    }
    CD_SUCCESS("Found yarp network.\n");

    teo::PlaybackLocomotion mod;
    return mod.runModule(rf);
}

