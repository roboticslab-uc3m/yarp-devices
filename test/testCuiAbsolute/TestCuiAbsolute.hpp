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

#include "TechnosoftIpos/TechnosoftIpos.hpp"    // -- uso de librería de drivers
#include "CuiAbsolute/CuiAbsolute.hpp"          // -- uso de librería de encoders absolutos

//-- Nuevos includes
#include <queue>


namespace teo
{

/**
 * @ingroup dumpCanBus //-- MODIFICAR
 *
 * @brief Launches one CAN bus driver, dumps output.
 *
 */

class TestCuiAbsolute : public yarp::os::RFModule, public yarp::os::Thread {
    public:
        TestCuiAbsolute();
        bool configure(yarp::os::ResourceFinder &rf);

        // -- Nuevas variables:        
        double firstTime;  // -- tiempo en el arranque (valor de tiempo aleatorio)
        double cleaningTime; // -- tiempo de espera para que no lleguen mensajes "basura" de encoders absolutos
        int nodeForReset;           // -- nodo que queremos resetear
        std::queue<int> queueIds;   // -- cola que almacenará los IDs

    protected:

        /** CAN BUS device. */
        yarp::dev::PolyDriver deviceDevCan0; // -- Dispositivo (HicoCan) que se crea.
        CanBusHico* iCanBus;

        /** CAN node object (canNodeDevice): Driver */
            yarp::dev::PolyDriver canNodeDriver;
            yarp::dev::IControlLimitsRaw* iControlLimitsRaw;
            yarp::dev::IControlModeRaw* iControlModeRaw;
            yarp::dev::IEncodersTimedRaw* iEncodersTimedRaw;
            yarp::dev::IPositionControlRaw* iPositionControlRaw;
            yarp::dev::IPositionDirectRaw* iPositionDirectRaw;
            yarp::dev::ITorqueControlRaw* iTorqueControlRaw;
            yarp::dev::IVelocityControlRaw* iVelocityControlRaw;
            ICanBusSharer* iCanBusSharer; // -- ??

        /** CAN node object (canNodeDevice): Cui Absolute Encoder. */
            yarp::dev::PolyDriver canNodeCuiAbsolute;
            CuiAbsolute* cuiAbsoluteEncoder;

        /** A helper function to display CAN messages. */
        std::string msgToStr(can_msg* message); // -- Muestra los mensajes que vienen del CAN

        // -- Funcion que envía mensajes CAN al encoder absoluto
        // .. se encuentra definica en CuiAbsolute/CuiAbsolute.hpp
        /*
         * Write message to the CAN buffer.
         * @param cob Message's COB
         * @param len Data field length
         * @param msgData Data to send
         * @return true/false on success/failure.

        bool send(uint32_t cob, uint16_t len, uint8_t * msgData);
        */

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

