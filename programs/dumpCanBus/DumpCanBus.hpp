// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __DUMP_CAN_BUS__
#define __DUMP_CAN_BUS__

#include <yarp/os/Bottle.h>
#include <yarp/os/Port.h>
#include <yarp/os/PortReaderBuffer.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/TypedReaderCallback.h>

#define DEFAULT_LOCAL_PORT "/dumpCanBus"

namespace roboticslab
{

/**
 * @ingroup dumpCanBus
 * @brief Connects to a remote CAN publisher port and dumps output.
 */
class DumpCanBus : public yarp::os::RFModule,
                   public yarp::os::TypedReaderCallback<yarp::os::Bottle>
{
public:
    ~DumpCanBus()
    { close(); }

    virtual bool configure(yarp::os::ResourceFinder & rf) override;

    virtual bool updateModule() override
    { return true; }

    virtual bool close() override;

    virtual void onRead(yarp::os::Bottle & b) override;

private:
    yarp::os::Port port;
    yarp::os::PortReaderBuffer<yarp::os::Bottle> portReader;
    bool useCanOpen;
};

} // namespace roboticslab

#endif // __DUMP_CAN_BUS__
