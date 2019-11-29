// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __CAN_RX_TH_THREADS_HPP__
#define __CAN_RX_TH_THREADS_HPP__

#include <limits>
#include <map>
#include <mutex>
#include <string>

#include <yarp/os/Thread.h>
#include <yarp/dev/CanBusInterface.h>

#include <ColorDebug.h>

#include "CanSenderDelegate.hpp"
#include "ICanBusSharer.hpp"

namespace roboticslab
{

/**
 * @ingroup CanBusControlboard
 */
class CanReaderWriterThread : public yarp::os::Thread
{
public:
    CanReaderWriterThread(const std::string & type, const std::string & id)
        : iCanBus(nullptr), iCanBufferFactory(nullptr), type(type), id(id), bufferSize(0), delay(0.0)
    {}

    virtual bool threadInit() override
    { canBuffer = iCanBufferFactory->createBuffer(bufferSize); return true; }

    virtual void threadRelease() override
    { iCanBufferFactory->destroyBuffer(canBuffer); }

    virtual void beforeStart() override
    { CD_INFO("Initializing CanBusControlboard %s thread %s.\n", type.c_str(), id.c_str()); }

    virtual void afterStart(bool success) override
    { CD_INFO("Configuring CanBusControlboard %s thread %s... %s\n", type.c_str(), id.c_str(), success ? "success" : "failure"); }

    virtual void onStop() override
    { CD_INFO("Stopping CanBusControlboard %s thread %s.\n", type.c_str(), id.c_str()); }

    virtual void setCanHandles(yarp::dev::ICanBus * iCanBus, yarp::dev::ICanBufferFactory * iCanBufferFactory, unsigned int bufferSize)
    { this->iCanBus = iCanBus; this->iCanBufferFactory = iCanBufferFactory; this->bufferSize = bufferSize; }

    void setDelay(double delay)
    { this->delay = delay <= 0 ? std::numeric_limits<double>::min() : delay; }

protected:
    yarp::dev::ICanBus * iCanBus;
    yarp::dev::ICanBufferFactory * iCanBufferFactory;
    yarp::dev::CanBuffer canBuffer;

    unsigned int bufferSize;
    double delay;

private:
    std::string type;
    std::string id;
};

/**
 * @ingroup CanBusControlboard
 */
class CanReaderThread : public CanReaderWriterThread
{
public:
    CanReaderThread(const std::string & id);
    void registerHandle(ICanBusSharer * p);
    virtual void run() override;

private:
    std::map<unsigned int, ICanBusSharer *> canIdToHandle;
};

/**
 * @ingroup CanBusControlboard
 */
class CanWriterThread : public CanReaderWriterThread
{
public:
    CanWriterThread(const std::string & id);
    ~CanWriterThread();

    virtual void run() override;
    virtual void setCanHandles(yarp::dev::ICanBus * iCanBus, yarp::dev::ICanBufferFactory * iCanBufferFactory, unsigned int bufferSize) override;
    CanSenderDelegate * getDelegate();

private:
    void handlePartialWrite(unsigned int sent);

    CanSenderDelegate * sender;
    unsigned int preparedMessages;
    mutable std::mutex bufferMutex;
};

} // namespace roboticslab

#endif // __CAN_RX_TH_THREADS_HPP__
