// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __DUMP_CAN_BUS__
#define __DUMP_CAN_BUS__

#include <string>

#include <yarp/os/ResourceFinder.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/Thread.h>

#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/CanBusInterface.h>

namespace roboticslab
{

/**
 * @ingroup dumpCanBus
 *
 * @brief Launches one CAN bus driver, dumps output.
 *
 */
class DumpCanBus : public yarp::os::RFModule, public yarp::os::Thread
{
public:
    DumpCanBus();
    bool configure(yarp::os::ResourceFinder &rf);

protected:

    yarp::dev::PolyDriver deviceDevCan0;
    yarp::dev::ICanBus* iCanBus;
    yarp::dev::ICanBufferFactory* iCanBufferFactory;
    yarp::dev::CanBuffer canInputBuffer;

    /** A helper function to display CAN messages. */
    std::string msgToStr(yarp::dev::CanMessage* message);
    double lastNow;

    virtual double getPeriod()
    {
        return 3.0;
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

#endif  // __DUMP_CAN_BUS__
