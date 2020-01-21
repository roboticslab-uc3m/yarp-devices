// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __CAN_BUS_BROKER_HPP__
#define __CAN_BUS_BROKER_HPP__

#include <mutex>
#include <string>

#include <yarp/os/Bottle.h>
#include <yarp/os/Port.h>
#include <yarp/os/PortReaderBuffer.h>
#include <yarp/os/PortWriterBuffer.h>
#include <yarp/os/RpcServer.h>
#include <yarp/os/Searchable.h>
#include <yarp/os/TypedReaderCallback.h>

#include <yarp/dev/CanBusInterface.h>
#include <yarp/dev/PolyDriver.h>

#include "CanRxTxThreads.hpp"
#include "SdoReplier.hpp"

namespace roboticslab
{

/**
 * @ingroup CanBusControlboard
 * @brief ...
 *
 * TODO: https://whatis.techtarget.com/definition/message-broker
 */
class CanBusBroker final : public yarp::os::TypedReaderCallback<yarp::os::Bottle>
{
public:
    CanBusBroker(const std::string & name);
    ~CanBusBroker();

    bool configure(const yarp::os::Searchable & config);
    bool registerDevice(yarp::dev::PolyDriver * driver);
    bool createPorts(const std::string & name);
    bool addFilters();
    bool clearFilters();
    bool startThreads();
    bool stopThreads();

    CanReaderThread * getReader() const
    { return readerThread; }

    CanWriterThread * getWriter() const
    { return writerThread; }

    std::string getName() const
    { return name; }

    virtual void onRead(yarp::os::Bottle & b) override;

private:
    std::string name;

    CanReaderThread * readerThread;
    CanWriterThread * writerThread;

    yarp::dev::ICanBus * iCanBus;
    yarp::dev::ICanBusErrors * iCanBusErrors;
    yarp::dev::ICanBufferFactory * iCanBufferFactory;

    yarp::os::Port dumpPort;
    yarp::os::PortWriterBuffer<yarp::os::Bottle> dumpWriter;
    std::mutex dumpMutex;

    yarp::os::Port sendPort;
    yarp::os::PortReaderBuffer<yarp::os::Bottle> commandReader;

    yarp::os::RpcServer sdoPort;
    SdoReplier sdoReplier;
};

} // namespace roboticslab

#endif // __CAN_BUS_BROKER_HPP__
