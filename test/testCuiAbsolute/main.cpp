// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/**
 *
 * @ingroup teo_body_programs
 * \defgroup dumpCanBus dumpCanBus
 *
 * @brief Creates an instance of teo::DumpCanBus.
 *
 * @section dumpCanBus_legal Legal
 *
 * Copyright: 2013 (C) Universidad Carlos III de Madrid
 *
 * Author: <a href="http://roboticslab.uc3m.es/roboticslab/people/r-de-santos">Raul de Santos Rico</a>
 *
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see license/LGPL.TXT
 *
 * @section testCuiAbsolute_install Installation
 *
 * The module is compiled when ENABLE_testCuiAbsolute is activated (default: OFF). For further
 * installation steps refer to <a class="el" href="pages.html">your own system installation guidelines</a>.
 *
 * @section testCuiAbsolute_running Running (assuming correct installation)
 *
 * First we must run a YARP name server if it is not running in our current namespace:
\verbatim
[on terminal 1] yarp server
\endverbatim
 * And then launch the actual module:
\verbatim
[on terminal 2] testCuiAbsolute
Pararmeters:
        --help (this help)		 --from [file.ini]	 --context [path]*	 --id[ID of Cui Absolute]
Modes: 	--startContinuousPublishing	--startPullPublishing	--stopPublishing
*can0: 	--canDevice /dev/can0		 can1: --canDevice /dev/can1
\endverbatim
 *
 * @section testCuiAbsolute_modify Modify
 *
 * This file can be edited at
 * programs/testCuiAbsolute/main.cpp
 *
 */

#include "TestCuiAbsolute.hpp"


YARP_DECLARE_PLUGINS(BodyYarp)

int main(int argc, char *argv[]) {

    YARP_REGISTER_PLUGINS(BodyYarp);

    yarp::os::ResourceFinder rf;
    rf.setVerbose(true); // -- Imprimir en pantalla la ubicación de los recursos a TRUE
    rf.setDefaultContext("testCuiAbsolute"); // -- Con esta función busca el directorio "testCuiAbsolute" dentro de build/share/
    rf.setDefaultConfigFile("testCuiAbsolute.ini"); // -- Nombre del fichero de configuración
    rf.configure(argc, argv);

    CD_INFO("Checking for yarp network...\n");
    yarp::os::Network yarp; // -- Red Yarp
    if (!yarp.checkNetwork()) { // -- Comprobación de la red Yarp (yarp server)
        CD_ERROR("Found no yarp network (try running \"yarpserver &\"), bye!\n");
        return 1;
    }
    CD_SUCCESS("Found yarp network.\n");

    teo::TestCuiAbsolute mod;      // -- Clase que hereda de RFModule y de Thread
    return mod.runModule(rf); // -- runModule llama a la función configure() de testCuiAbsolute.hpp
}

