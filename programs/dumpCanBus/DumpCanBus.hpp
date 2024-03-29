// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __DUMP_CAN_BUS_HPP__
#define __DUMP_CAN_BUS_HPP__

#include <yarp/os/Bottle.h>
#include <yarp/os/Port.h>
#include <yarp/os/PortReaderBuffer.h>
#include <yarp/os/Stamp.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/TypedReaderCallback.h>

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
    ~DumpCanBus() override
    { close(); }

    bool configure(yarp::os::ResourceFinder & rf) override;

    bool updateModule() override
    { return true; }

    bool close() override;

    void onRead(yarp::os::Bottle & b) override;

private:
    void printMessage(const yarp::os::Bottle & b, const yarp::os::Stamp & stamp);

    yarp::os::Port port;
    yarp::os::PortReaderBuffer<yarp::os::Bottle> portReader;
    bool useCanOpen;
    bool printTimestamp;
};

} // namespace roboticslab

#endif // __DUMP_CAN_BUS_HPP__
