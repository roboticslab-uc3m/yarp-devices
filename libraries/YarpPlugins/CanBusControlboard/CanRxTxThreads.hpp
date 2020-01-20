// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __CAN_RX_TH_THREADS_HPP__
#define __CAN_RX_TH_THREADS_HPP__

#include <mutex>
#include <string>
#include <unordered_map>

#include <yarp/os/Bottle.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/Thread.h>
#include <yarp/dev/CanBusInterface.h>

#include "CanSenderDelegate.hpp"
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
        : iCanBus(nullptr), iCanBusErrors(nullptr), iCanBufferFactory(nullptr), dumpPort(nullptr),
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

    void attachDumpPort(yarp::os::BufferedPort<yarp::os::Bottle> * dumpPort)
    { this->dumpPort = dumpPort; }

protected:
    yarp::dev::ICanBus * iCanBus;
    yarp::dev::ICanBusErrors * iCanBusErrors;
    yarp::dev::ICanBufferFactory * iCanBufferFactory;
    yarp::dev::CanBuffer canBuffer;

    yarp::os::BufferedPort<yarp::os::Bottle> * dumpPort;

    unsigned int bufferSize;
    double delay;

private:
    std::string type;
    std::string id;
};

/**
 * @ingroup CanBusControlboard
 * @brief A thread that attends CAN reads.
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

    //! Retrieve internal map of CAN handles.
    const std::unordered_map<unsigned int, ICanBusSharer *> & getHandleMap()
    { return canIdToHandle; }

    virtual void run() override;

private:
    std::unordered_map<unsigned int, ICanBusSharer *> canIdToHandle;
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

    virtual void setCanHandles(yarp::dev::ICanBus * iCanBus, yarp::dev::ICanBusErrors * iCanBusErrors,
            yarp::dev::ICanBufferFactory * iCanBufferFactory) override;

    virtual void run() override;

private:
    //! In case a write did not succeed, rearrange the CAN message buffer.
    void handlePartialWrite(unsigned int sent);

    CanSenderDelegate * sender;
    unsigned int preparedMessages;
    mutable std::mutex bufferMutex;
};

} // namespace roboticslab

#endif // __CAN_RX_TH_THREADS_HPP__
