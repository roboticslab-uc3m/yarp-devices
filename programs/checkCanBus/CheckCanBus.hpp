// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __CHECK_CAN_BUS__
#define __CHECK_CAN_BUS__

#include <queue>
#include <string>

#include <yarp/os/ResourceFinder.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/Thread.h>

#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/dev/CanBusInterface.h>

#include "ICanBusSharer.h"
#include "ITechnosoftIpos.h"

namespace roboticslab
{

/**
 * @ingroup dumpCanBus //-- MODIFICAR
 *
 * @brief Launches one CAN bus driver, dumps output.
 *
 */

class CheckCanBus : public yarp::os::RFModule, public yarp::os::Thread
{
public:
    CheckCanBus();
    bool configure(yarp::os::ResourceFinder &rf);

    // -- Nuevas variables:
    double timeOut;     // -- tiempo de espera para comprobar el ID (s)
    double firstTime;  // -- tiempo en el arranque (valor de tiempo aleatorio)
    double cleaningTime; // -- tiempo de espera para que no lleguen mensajes "basura" de encoders absolutos
    int nodeForReset;           // -- nodo que queremos resetear
    std::queue<int> queueIds;   // -- cola que almacenará los IDs

protected:

    /** CAN BUS device. */
    yarp::dev::PolyDriver deviceDevCan0; // -- Dispositivo (HicoCan) que se crea.
    yarp::dev::ICanBus* iCanBus;
    yarp::dev::ICanBufferFactory* iCanBufferFactory;
    yarp::dev::CanBuffer canInputBuffer;

    /** CAN node object. */
    yarp::dev::PolyDriver canNodeDevice;
    yarp::dev::IControlLimitsRaw* iControlLimitsRaw;
    yarp::dev::IControlModeRaw* iControlModeRaw;
    yarp::dev::IEncodersTimedRaw* iEncodersTimedRaw;
    yarp::dev::IPositionControlRaw* iPositionControlRaw;
    yarp::dev::IPositionDirectRaw* iPositionDirectRaw;
    yarp::dev::ITorqueControlRaw* iTorqueControlRaw;
    yarp::dev::IVelocityControlRaw* iVelocityControlRaw;
    ICanBusSharer* iCanBusSharer; // -- ??
    ITechnosoftIpos* technosoftIpos;    //-- ok practice?

    /** A helper function to display CAN messages. */
    std::string msgToStr(yarp::dev::CanMessage* message); // -- Muestra los mensajes que vienen del CAN

    // -- Funcion que se encargará de chekear los IDs introducidos e imprimir los detectados
    void checkIds(yarp::dev::CanMessage* message); // --Declara función que encontraremos en el .hpp

    // -- Funcion que se encargará de imprimir los IDs no detectados
    void printWronglIds();

    double lastNow; // -- Muestra el tiempo actual

    virtual double getPeriod()
    {
        return 3.0;   // Periodicidad de llamada a updateModule en [s]
    }
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

}  // namespace roboticslab

#endif  // __CHECK_CAN_BUS__
