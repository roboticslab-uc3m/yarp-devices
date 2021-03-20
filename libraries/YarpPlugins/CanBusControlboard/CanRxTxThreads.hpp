// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __CAN_RX_TH_THREADS_HPP__
#define __CAN_RX_TH_THREADS_HPP__

#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

#include <yarp/os/Bottle.h>
#include <yarp/os/PortWriterBuffer.h>
#include <yarp/os/Thread.h>

#include <yarp/dev/CanBusInterface.h>

#include "ICanBusSharer.hpp"

namespace roboticslab
{

/**
 * @ingroup CanBusControlboard
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
        : iCanBus(nullptr), iCanBusErrors(nullptr), iCanBufferFactory(nullptr),
          dumpWriter(nullptr), dumpMutex(nullptr), busLoadMonitor(nullptr),
          bufferSize(bufferSize), delay(delay), type(type), id(id)
    { }

    //! Virtual destructor.
    virtual ~CanReaderWriterThread() = default;

    //! Invoked by the thread right before it is started.
    virtual bool threadInit() override
    { canBuffer = iCanBufferFactory->createBuffer(bufferSize); return true; }

    //! Invoked by the thread right after it is started.
    virtual void threadRelease() override
    { iCanBufferFactory->destroyBuffer(canBuffer); }

    //! Invoked by the caller right before the thread is started.
    virtual void beforeStart() override;

    //! Invoked by the caller right before the thread is joined.
    virtual void afterStart(bool success) override;

    //! Callback on thread stop.
    virtual void onStop() override;

    //! The thread will invoke this once.
    virtual void run() override = 0;

    //! Configure CAN interface handles.
    virtual void setCanHandles(yarp::dev::ICanBus * iCanBus, yarp::dev::ICanBusErrors * iCanBusErrors,
            yarp::dev::ICanBufferFactory * iCanBufferFactory)
    {
        this->iCanBus = iCanBus; this->iCanBusErrors = iCanBusErrors; this->iCanBufferFactory = iCanBufferFactory;
    }

    //! Attach YARP port writer for CAN message dumping.
    void attachDumpWriter(yarp::os::PortWriterBuffer<yarp::os::Bottle> * dumpWriter, std::mutex * dumpMutex)
    { this->dumpWriter = dumpWriter; this->dumpMutex = dumpMutex; }

    //! Attach CAN bus load monitor.
    void attachBusLoadMonitor(CanMessageNotifier * busLoadMonitor)
    { this->busLoadMonitor = busLoadMonitor; }

protected:
    //! Dump a CAN message to a YARP bottle.
    static void dumpMessage(const can_message & msg, yarp::os::Bottle & b);

    yarp::dev::ICanBus * iCanBus;
    yarp::dev::ICanBusErrors * iCanBusErrors;
    yarp::dev::ICanBufferFactory * iCanBufferFactory;
    yarp::dev::CanBuffer canBuffer;

    yarp::os::PortWriterBuffer<yarp::os::Bottle> * dumpWriter;
    std::mutex * dumpMutex;

    CanMessageNotifier * busLoadMonitor;

    unsigned int bufferSize;
    double delay;

private:
    std::string type;
    std::string id;
};

/**
 * @ingroup CanBusControlboard
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
    const std::vector<ICanBusSharer *> & getHandles()
    { return handles; }

    //! Attach custom CAN message responder handle.
    void attachCanNotifier(CanMessageNotifier * canMessageNotifier)
    { this->canMessageNotifier = canMessageNotifier; }

    virtual void run() override;

private:
    std::vector<ICanBusSharer *> handles;
    std::unordered_map<unsigned int, ICanBusSharer *> canIdToHandle;
    CanMessageNotifier * canMessageNotifier;
};

/**
 * @ingroup CanBusControlboard
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
    virtual ~CanWriterThread();

    //! Retrieve a handle to the CAN sender delegate.
    CanSenderDelegate * getDelegate();

    //! Send awaiting messages and clear the queue.
    void flush();

    virtual void run() override;

private:
    //! In case a write did not succeed, rearrange the CAN message buffer.
    void handlePartialWrite(unsigned int sent);

    unsigned int preparedMessages;
    CanSenderDelegate * sender;
    mutable std::mutex bufferMutex;
};

} // namespace roboticslab

#endif // __CAN_RX_TH_THREADS_HPP__
