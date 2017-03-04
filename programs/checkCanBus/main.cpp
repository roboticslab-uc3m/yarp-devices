// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/**
 *
 * @ingroup yarp_devices_programs
 * \defgroup checkCanBus checkCanBus
 *
 * @brief Creates an instance of teo::CheckCanBus.
 *
 * @section checkCanBus_legal Legal
 *
 * Copyright: 2016 (C) Universidad Carlos III de Madrid
 *
 * Author: <a href="http://roboticslab.uc3m.es/roboticslab/people/r-de-santos">Raul de Santos Rico</a>
 *
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see license/LGPL.TXT
 *
 * @section checkCanBus_install Installation
 *
 * The module is compiled when ENABLE_checkCanBus is activated (default: ON). For further
 * installation steps refer to <a class="el" href="pages.html">your own system installation guidelines</a>.
 *
 * @section checkCanBus_running Running (assuming correct installation)
 *
 * First we must run a YARP name server if it is not running in our current namespace:
\verbatim
[on terminal 1] yarp server
\endverbatim
 * And then launch the actual module:
\verbatim
[on terminal 2] checkCanBus
Parameters:
        --help (this help)	 --ids ["(id)"] 		 --from [file.ini]	 --canDevice [path]*
        --timeOut [s]		 --resetAll (for all nodes)	 --resetNode [node]	 --cleaningTime [s]

*can0: 	--canDevice /dev/can0		 can1: --canDevice /dev/can1
 .ini:	 checkLocomotionCan0.ini	 checkLocomotionCan1.ini	 checkManipulationCan0.ini	 checkManipulationCan1.ini

Example of uses:
* Reset driver ID [23] and check it:				 checkCanBus --canDevice /dev/can1 --ids 23 --resetNode 23
* Reset drivers IDs [23,24] and check them:			 checkCanBus --canDevice /dev/can1 --ids "(23 24)" --resetAll
* Reset all drivers of manipulation Can0 and check devices:	 checkCanBus --canDevice /dev/can0 --from checkManipulationCan0.ini --resetAll

\endverbatim
 *
 * @section checkCanBus_modify Modify
 *
 * This file can be edited at
 * programs/checkCanBus/main.cpp
 *
 */

#include "CheckCanBus.hpp"


// YARP_DECLARE_PLUGINS(BodyYarp)

int main(int argc, char *argv[])
{

//    YARP_REGISTER_PLUGINS(BodyYarp);

    yarp::os::ResourceFinder rf;
    rf.setVerbose(true); // -- Imprimir en pantalla la ubicación de los recursos a TRUE
    rf.setDefaultContext("checkCanBus"); // -- Con esta función busca el directorio "checkCanBus" dentro de build/share/
    rf.setDefaultConfigFile("checkCanBus.ini"); // -- Nombre del fichero de configuración
    rf.configure(argc, argv);

    CD_INFO("Checking for yarp network...\n");
    yarp::os::Network yarp; // -- Red Yarp
    if (!yarp.checkNetwork())   // -- Comprobación de la red Yarp (yarp server)
    {
        CD_ERROR("Found no yarp network (try running \"yarpserver &\"), bye!\n");
        return 1;
    }
    CD_SUCCESS("Found yarp network.\n");

    teo::CheckCanBus mod;      // -- Clase que hereda de RFModule y de Thread
    return mod.runModule(rf); // -- runModule llama a la función configure() de CheckCanBus.hpp
}

