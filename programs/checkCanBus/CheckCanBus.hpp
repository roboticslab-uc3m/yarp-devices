// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __CHECK_CAN_BUS__
#define __CHECK_CAN_BUS__

#include <yarp/os/RFModule.h>
#include <yarp/os/Module.h>
#include <yarp/os/Network.h>
#include <yarp/os/Port.h>
#include <yarp/os/BufferedPort.h>

#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/Wrapper.h>

#include <string>
#include <stdlib.h>

#include "ICanBusSharer.h"
#include "ColorDebug.hpp"

//-- Nuevos includes
//#include <vector>
#include <queue>


namespace teo
{

/**
 * @ingroup dumpCanBus //-- MODIFICAR
 *
 * @brief Launches one CAN bus driver, dumps output.
 *
 */

class CheckCanBus : public yarp::os::RFModule, public yarp::os::Thread {
    public:
        CheckCanBus();
        bool configure(yarp::os::ResourceFinder &rf);

        // -- Nuevas variables:
        double timeOut;     // -- tiempo de espera para comprobar el ID (s)
        double firstTime;  // -- tiempo en el arranque (valor de tiempo aleatorio)
        //std::vector<int> vectorIds;    // -- vector que almacenará los IDs y su activación
        std::queue<int> queueIds;   // -- cola que almacenará los IDs

    protected:

        yarp::dev::PolyDriver deviceDevCan0; // -- Dispositivo que se crea. Hace referencia al dispositivo HicoCan
        CanBusHico* iCanBus;

        /** A helper function to display CAN messages. */
        std::string msgToStr(can_msg* message); // -- Muestra los mensajes que vienen del CAN

        // -- Funcion que se encargará de chekear los IDs introducidos e imprimir los detectados
        void checkIds(can_msg* message); // --Declara función que encontraremos en el .hpp

        // -- Funcion que se encargará de imprimir los IDs no detectados
        void printWronglIds();

        double lastNow; // -- Muestra el tiempo actual

        virtual double getPeriod() {return 3.0;}  // Periodicidad de llamada a updateModule en [s]
        virtual bool updateModule();
        virtual bool close();

//        virtual bool interruptModule();
//        virtual int period;

    // -------- Thread declarations. Implementation in ThreadImpl.cpp --------

        /**
         * Main body of the new thread.
         * Override this method to do what you want.
         * After Thread::start is called, this
         * method will start running in a separate thread.
         * It is important that this method either keeps checking
         * Thread::isStopping to see if it should stop, or
         * you override the Thread::onStop method to interact
         * with it in some way to shut the new thread down.
         * There is no really reliable, portable way to stop
         * a thread cleanly unless that thread cooperates.
         */
        virtual void run();
};

}  // namespace teo

#endif  // __CHECK_CAN_BUS__

