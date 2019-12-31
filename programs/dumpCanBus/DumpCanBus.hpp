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
 * @brief Launches one CAN bus driver, dumps output.
 */
class DumpCanBus : public yarp::os::RFModule,
                   public yarp::os::Thread
{
public:

    DumpCanBus();

    virtual bool configure(yarp::os::ResourceFinder & rf);
    virtual bool updateModule()
    { return true; }
    virtual bool close();

    // -------- Thread declarations. Implementation in ThreadImpl.cpp --------
    virtual void run();

private:

    yarp::dev::PolyDriver canDevice;
    yarp::dev::ICanBus * iCanBus;
    yarp::dev::ICanBufferFactory * iCanBufferFactory;
    yarp::dev::CanBuffer canInputBuffer;
};

} // namespace roboticslab

#endif // __DUMP_CAN_BUS__
