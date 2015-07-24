// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __DUMP_CAN_BUS__
#define __DUMP_CAN_BUS__

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

using namespace yarp::os;
using namespace yarp::dev;

namespace teo
{

/**
 * @ingroup DumpCanBus
 *
 * @brief Launches one CAN bus driver, dumps output.
 *
 */
class DumpCanBus : public RFModule, public Thread {
    public:
        DumpCanBus();
        bool configure(ResourceFinder &rf);

    protected:
        yarp::dev::PolyDriver deviceDevCan0;
        CanBusHico* iCanBus;

        /** A helper function to display CAN messages. */
        std::string msgToStr(can_msg* message);

        virtual double getPeriod() {return 3.0;}
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

#endif  // __DUMP_CAN_BUS__

