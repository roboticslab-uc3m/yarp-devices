// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __CAN_RX_TH_THREADS_HPP__
#define __CAN_RX_TH_THREADS_HPP__

#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

#include <yarp/os/Bottle.h>
#include <yarp/os/Contactable.h>
#include <yarp/os/PortWriterBuffer.h>
#include <yarp/os/Stamp.h>
#include <yarp/os/Thread.h>

#include <yarp/dev/CanBusInterface.h>

#include "ICanBusSharer.hpp"

namespace roboticslab
{

/**
 * @ingroup CanBusBroker
 * @brief Base class for a thread that attends CAN reads or writes.
 *
 * Child classes take advantage of CAN message buffers to perform bulk reads
 * and writes.
 */
class CanReaderWriterThread : public yarp::os::Thread
{
public:
    //! Constructor.
    CanReaderWriterThread(const std::string & type, const std::string & id, double delay, unsigned int bufferSize)
        : iCanBus(nullptr), iCanBufferFactory(nullptr),
          dumpPort(nullptr), dumpWriter(nullptr), dumpMutex(nullptr), busLoadMonitor(nullptr),
          bufferSize(bufferSize), delay(delay), type(type), id(id)
    { }

    //! Virtual destructor.
    ~CanReaderWriterThread() override = default;

    //! Invoked by the thread right before it is started.
    bool threadInit() override
    { canBuffer = iCanBufferFactory->createBuffer(bufferSize); return true; }

    //! Invoked by the thread right after it is started.
    void threadRelease() override
    { iCanBufferFactory->destroyBuffer(canBuffer); }

    //! Invoked by the caller right before the thread is started.
    void beforeStart() override;

    //! Invoked by the caller right before the thread is joined.
    void afterStart(bool success) override;

    //! Callback on thread stop.
    void onStop() override;

    //! Configure CAN interface handles.
    void setCanHandles(yarp::dev::ICanBus * iCanBus, yarp::dev::ICanBufferFactory * iCanBufferFactory)
    {
        this->iCanBus = iCanBus; this->iCanBufferFactory = iCanBufferFactory;
    }

    //! Attach YARP port writer for CAN message dumping.
    void attachDumpWriter(yarp::os::Contactable * dumpPort,
                          yarp::os::PortWriterBuffer<yarp::os::Bottle> * dumpWriter,
                          std::mutex * dumpMutex)
    {
        this->dumpPort = dumpPort; this->dumpWriter = dumpWriter; this->dumpMutex = dumpMutex;
    }

    //! Attach CAN bus load monitor.
    void attachBusLoadMonitor(ICanMessageNotifier * busLoadMonitor)
    { this->busLoadMonitor = busLoadMonitor; }

protected:
    //! Dump a CAN message to a YARP bottle.
    static void dumpMessage(const can_message & msg, yarp::os::Bottle & b);

    yarp::dev::ICanBus * iCanBus;
    yarp::dev::ICanBufferFactory * iCanBufferFactory;
    yarp::dev::CanBuffer canBuffer;

    yarp::os::Contactable * dumpPort;
    yarp::os::PortWriterBuffer<yarp::os::Bottle> * dumpWriter;
    std::mutex * dumpMutex;

    yarp::os::Stamp lastStamp;

    ICanMessageNotifier * busLoadMonitor;

    unsigned int bufferSize;
    double delay;

private:
    std::string type;
    std::string id;
};

/**
 * @ingroup CanBusBroker
 * @brief A thread that deals with CAN reads.
 *
 * Messages are forwarded to each raw subdevice given the CAN node ID.
 */
class CanReaderThread : public CanReaderWriterThread
{
public:
    //! Constructor.
    CanReaderThread(const std::string & id, double delay, unsigned int bufferSize);

    //! Map CAN node ids with handles.
    void registerHandle(ICanBusSharer * p);

    //! Retrieve collection of handles to wrapped CAN devices.
    const std::vector<ICanBusSharer *> & getHandles() const
    { return handles; }

    //! Attach custom CAN message responder handle.
    void attachCanNotifier(ICanMessageNotifier * canMessageNotifier)
    { this->canMessageNotifier = canMessageNotifier; }

    void run() override;

private:
    std::vector<ICanBusSharer *> handles;
    std::unordered_map<unsigned int, ICanBusSharer *> canIdToHandle;
    ICanMessageNotifier * canMessageNotifier;
};

/**
 * @ingroup CanBusBroker
 * @brief A thread that attends CAN writes.
 *
 * Uses @ref YarpCanSenderDelegate to let raw subdevices register outgoing CAN
 * messages. Those are written to the CAN network in FIFO batches on each step.
 */
class CanWriterThread : public CanReaderWriterThread
{
public:
    //! Constructor.
    CanWriterThread(const std::string & id, double delay, unsigned int bufferSize);

    //! Destructor.
    ~CanWriterThread() override;

    //! Retrieve a handle to the CAN sender delegate.
    ICanSenderDelegate * getDelegate()
    { return sender; }

    //! Send awaiting messages and clear the queue.
    void flush();

    void run() override;

private:
    //! In case a write did not succeed, rearrange the CAN message buffer.
    void handlePartialWrite(unsigned int sent);

    unsigned int preparedMessages;
    ICanSenderDelegate * sender;
    mutable std::mutex bufferMutex;
};

} // namespace roboticslab

#endif // __CAN_RX_TH_THREADS_HPP__
