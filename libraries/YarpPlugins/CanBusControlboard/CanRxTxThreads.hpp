// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __CAN_RX_TH_THREADS_HPP__
#define __CAN_RX_TH_THREADS_HPP__

#include <map>
#include <vector>

#include <yarp/os/Thread.h>
#include <yarp/dev/CanBusInterface.h>

#include "ICanBusSharer.h"

namespace roboticslab
{

/**
 * @ingroup CanBusControlboard
 */
class CanReaderWriterThread : public yarp::os::Thread
{
public:
    CanReaderWriterThread() : iCanBus(0), iCanBufferFactory(0), bufferSize(0)
    {}

    virtual void run() = 0;

    virtual bool threadInit()
    { canBuffer = iCanBufferFactory->createBuffer(bufferSize); return true; }

    virtual void threadRelease()
    { iCanBufferFactory->destroyBuffer(canBuffer); }

    void setCanHandles(yarp::dev::ICanBus * iCanBus, yarp::dev::ICanBufferFactory * iCanBufferFactory, int bufferSize)
    { this->iCanBus = iCanBus; this->iCanBufferFactory = iCanBufferFactory; this->bufferSize = bufferSize; }

protected:
    yarp::dev::ICanBus * iCanBus;
    yarp::dev::ICanBufferFactory * iCanBufferFactory;
    yarp::dev::CanBuffer canBuffer;

    int bufferSize;
};

/**
 * @ingroup CanBusControlboard
 */
class CanReaderThread : public CanReaderWriterThread
{
public:
    CanReaderThread(const std::map<int, int> & idxFromCanId, const std::vector<ICanBusSharer *> & iCanBusSharer);
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
    virtual void run();
};

} // namespace roboticslab

#endif // __CAN_RX_TH_THREADS_HPP__
