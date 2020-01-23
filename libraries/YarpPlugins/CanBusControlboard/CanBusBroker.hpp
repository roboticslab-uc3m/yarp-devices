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
 * @brief Message broker of a CAN bus with YARP port intefaces.
 *
 * Instantiates and orchestrates independent CAN read/write threads and manages
 * message buffers. Outgoing messages are loaded into a queue and processed in
 * batches, incoming messages are forwarded to registered listeners.
 *
 * CAN traffic is interfaced via optional YARP ports to allow remote access.
 * This includes an output dump port, an input command port, and an RPC service
 * for confirmed SDO transfers.
 */
class CanBusBroker final : public yarp::os::TypedReaderCallback<yarp::os::Bottle>
{
public:
    //! Constructor, passes string identifier of the CAN bus.
    CanBusBroker(const std::string & name);

    //! Destructor.
    ~CanBusBroker();

    //! Configure this CAN bus.
    bool configure(const yarp::os::Searchable & config);

    //! Register CAN handles associated to the input device driver.
    bool registerDevice(yarp::dev::PolyDriver * driver);

    //! Set CAN acceptance filters.
    bool addFilters();

    //! Clear CAN acceptance filters.
    bool clearFilters();

    //! Start CAN read/write threads.
    bool startThreads();

    //! Stop CAN read/write threads.
    bool stopThreads();

    //! Get handle of the CAN RX thread.
    CanReaderThread * getReader() const
    { return readerThread; }

    //! Get handle
    CanWriterThread * getWriter() const
    { return writerThread; }

    //! Retrieve string identifier for this CAN bus.
    std::string getName() const
    { return name; }

    //! Callback on incoming remote CAN commands.
    virtual void onRead(yarp::os::Bottle & b) override;

private:
    //! Open remote CAN interface ports.
    bool createPorts(const std::string & prefix);

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
