// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __CAN_RX_TH_THREADS_HPP__
#define __CAN_RX_TH_THREADS_HPP__

#include <map>
#include <mutex>
#include <string>
#include <vector>

#include <yarp/os/Thread.h>
#include <yarp/dev/CanBusInterface.h>

#include <ColorDebug.h>

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
        : iCanBus(0), iCanBufferFactory(0), type(type), id(id), bufferSize(0)
    {}

    virtual void run() = 0;

    virtual bool threadInit()
    { canBuffer = iCanBufferFactory->createBuffer(bufferSize); return true; }

    virtual void threadRelease()
    { iCanBufferFactory->destroyBuffer(canBuffer); }

    virtual void beforeStart()
    { CD_INFO("Initializing CanBusControlboard %s thread %s.\n", type.c_str(), id.c_str()); }

    virtual void afterStart(bool success)
    { CD_INFO("Configuring CanBusControlboard %s thread %s... %s\n", type.c_str(), id.c_str(), success ? "success" : "failure"); }

    virtual void onStop()
    { CD_INFO("Stopping CanBusControlboard %s thread %s.\n", type.c_str(), id.c_str()); }

    virtual void setCanHandles(yarp::dev::ICanBus * iCanBus, yarp::dev::ICanBufferFactory * iCanBufferFactory, int bufferSize)
    { this->iCanBus = iCanBus; this->iCanBufferFactory = iCanBufferFactory; this->bufferSize = bufferSize; }

protected:
    yarp::dev::ICanBus * iCanBus;
    yarp::dev::ICanBufferFactory * iCanBufferFactory;
    yarp::dev::CanBuffer canBuffer;

    int bufferSize;

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
    CanReaderThread(const std::string & id,
            const std::map<int, int> & idxFromCanId,
            const std::vector<ICanBusSharer *> & iCanBusSharer);

    virtual void run();

private:
    const std::map<int, int> & idxFromCanId;
    const std::vector<ICanBusSharer *> & iCanBusSharer;
};

/**
 * @ingroup CanBusControlboard
 */
class CanWriterThread : public CanReaderWriterThread
{
public:
    CanWriterThread(const std::string & id);
    ~CanWriterThread();

    virtual void run();
    virtual void setCanHandles(yarp::dev::ICanBus * iCanBus, yarp::dev::ICanBufferFactory * iCanBufferFactory, int bufferSize);
    CanSenderDelegate * getDelegate();

private:
    CanSenderDelegate * sender;
    int preparedMessages;
    mutable std::mutex bufferMutex;
};

} // namespace roboticslab

#endif // __CAN_RX_TH_THREADS_HPP__
