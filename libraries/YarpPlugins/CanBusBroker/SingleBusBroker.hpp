// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __SINGLE_BUS_BROKER_HPP__
#define __SINGLE_BUS_BROKER_HPP__

#include <atomic>
#include <mutex>
#include <string>
#include <vector>

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
#include "BusLoadMonitor.hpp"

namespace roboticslab
{

/**
 * @ingroup CanBusBroker
 * @brief Message broker of a CAN bus with YARP port interfaces.
 *
 * Instantiates and orchestrates independent CAN read/write threads and manages
 * message buffers. Outgoing messages are loaded into a queue and processed in
 * batches, incoming messages are forwarded to registered listeners.
 *
 * CAN traffic is interfaced via optional YARP ports to allow remote access.
 * This includes an output dump port, an input command port, and an RPC service
 * for confirmed SDO transfers.
 */
class SingleBusBroker final : public yarp::os::TypedReaderCallback<yarp::os::Bottle>
{
public:
    //! Constructor, accepts the string identifiers of the CAN bus and its nodes.
    SingleBusBroker(const std::string & _busName, const std::vector<std::string> & _nodeNames)
        : busName(_busName), nodeNames(_nodeNames)
    { }

    //! Destructor.
    ~SingleBusBroker() override;

    //! Configure this CAN bus.
    bool configure(const yarp::os::Searchable & config);

    //! Register CAN handles associated to the input device driver.
    bool registerDevice(yarp::dev::PolyDriver * driver);

    //! Clear CAN acceptance filters.
    bool clearFilters();

    //! Start CAN read/write threads.
    bool startThreads();

    //! Stop CAN read/write threads.
    bool stopThreads();

    //! Get handle of the CAN RX thread.
    CanReaderThread * getReader() const
    { return readerThread; }

    //! Get handle of the CAN TX thread.
    CanWriterThread * getWriter() const
    { return writerThread; }

    //! Retrieve string identifier for this CAN bus.
    const std::string & getBusName() const
    { return busName; }

    //! Retrieve string identifiers for the nodes of this CAN bus.
    const std::vector<std::string> & getNodeNames() const
    { return nodeNames; }

    //! Callback on incoming remote CAN commands.
    void onRead(yarp::os::Bottle & b) override;

    //! Mark this instance as fully initialized or not.
    void markInitialized(bool initialized)
    { this->initialized = initialized; }

    //! Check if this instance is fully initialized.
    bool isInitialized() const
    { return initialized; }

private:
    //! Open remote CAN interface ports.
    bool createPorts(const std::string & prefix);

    std::string busName;
    std::vector<std::string> nodeNames;

    CanReaderThread * readerThread {nullptr};
    CanWriterThread * writerThread {nullptr};

    yarp::dev::ICanBus * iCanBus {nullptr};
    yarp::dev::ICanBufferFactory * iCanBufferFactory {nullptr};

    yarp::os::Port dumpPort;
    yarp::os::PortWriterBuffer<yarp::os::Bottle> dumpWriter;
    std::mutex dumpMutex;

    yarp::os::Port sendPort;
    yarp::os::PortReaderBuffer<yarp::os::Bottle> commandReader;

    yarp::os::RpcServer sdoPort;
    SdoReplier sdoReplier;

    yarp::os::Port busLoadPort;
    BusLoadMonitor * busLoadMonitor {nullptr};

    bool registered {false};
    std::atomic_bool initialized {false};
};

} // namespace roboticslab

#endif // __SINGLE_BUS_BROKER_HPP__
