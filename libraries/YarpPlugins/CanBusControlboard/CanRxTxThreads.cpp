// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "CanRxTxThreads.hpp"

#include <cstring>

#include <yarp/os/Time.h>

#include <ColorDebug.h>

#include "YarpCanSenderDelegate.hpp"
#include "CanBusBroker.hpp"

using namespace roboticslab;

// -----------------------------------------------------------------------------

void CanReaderWriterThread::beforeStart()
{
    CD_INFO("Initializing CanBusControlboard %s thread %s.\n", type.c_str(), id.c_str());
}

// -----------------------------------------------------------------------------

void CanReaderWriterThread::afterStart(bool success)
{
    CD_INFO("Configuring CanBusControlboard %s thread %s... %s\n", type.c_str(), id.c_str(), success ? "success" : "failure");
}

// -----------------------------------------------------------------------------

void CanReaderWriterThread::onStop()
{
    CD_INFO("Stopping CanBusControlboard %s thread %s.\n", type.c_str(), id.c_str());
}

// -----------------------------------------------------------------------------

void CanReaderWriterThread::dumpMessage(const yarp::dev::CanMessage & msg)
{
    std::lock_guard<std::mutex> lock(*dumpMutex);

    yarp::os::Bottle & b = dumpWriter->prepare();
    b.clear();
    b.addInt16(msg.getId());

    if (msg.getLen() != 0)
    {
        yarp::os::Bottle & data = b.addList();

        for (int j = 0; j < msg.getLen(); j++)
        {
            data.addInt8(msg.getData()[j]);
        }
    }

    dumpWriter->write();
}

// -----------------------------------------------------------------------------

CanReaderThread::CanReaderThread(const std::string & id, double delay, unsigned int bufferSize)
    : CanReaderWriterThread("read", id, delay, bufferSize),
      canMessageNotifier(nullptr)
{ }

// -----------------------------------------------------------------------------

void CanReaderThread::registerHandle(ICanBusSharer * p)
{
    canIdToHandle[p->getId()] = p;

    for (auto id : p->getAdditionalIds())
    {
        canIdToHandle[id] = p;
    }
}

// -----------------------------------------------------------------------------

void CanReaderThread::run()
{
    unsigned int read;
    bool ok;

    while (!isStopping())
    {
        //-- Lend CPU time to write threads.
        // https://github.com/roboticslab-uc3m/yarp-devices/issues/191
        yarp::os::Time::delay(delay);

        //-- Return immediately if there is nothing to be read (non-blocking call), return false on errors.
        ok = iCanBus->canRead(canBuffer, bufferSize, &read);

        //-- All debugging messages should be contained in canRead, so just loop again.
        if (!ok || read == 0) continue;

        for (int i = 0; i < read; i++)
        {
            const yarp::dev::CanMessage & msg = canBuffer[i];
            auto it = canIdToHandle.find(msg.getId() & 0x7F);

            if (it != canIdToHandle.end())
            {
                it->second->notifyMessage({msg.getId(), msg.getLen(), msg.getData()});
            }

            if (dumpWriter)
            {
                dumpMessage(msg);
            }

            if (canMessageNotifier)
            {
                canMessageNotifier->notifyMessage({msg.getId(), msg.getLen(), msg.getData()});
            }
        }
    }
}

// -----------------------------------------------------------------------------

CanWriterThread::CanWriterThread(const std::string & id, double delay, unsigned int bufferSize)
    : CanReaderWriterThread("write", id, delay, bufferSize),
      preparedMessages(0),
      sender(new YarpCanSenderDelegate(canBuffer, bufferMutex, preparedMessages, bufferSize))
{ }

// -----------------------------------------------------------------------------

CanWriterThread::~CanWriterThread()
{
    delete sender;
}

// -----------------------------------------------------------------------------

void CanWriterThread::flush()
{
    std::lock_guard<std::mutex> lock(bufferMutex);

    //-- Nothing to write, exit.
    if (preparedMessages == 0) return;

    yarp::dev::CanErrors errors;

    //-- Query bus state.
    if (!iCanBusErrors->canGetErrors(errors) || errors.busoff)
    {
        //-- Bus off, reset TX queue.
        preparedMessages = 0;
        return;
    }

    unsigned int sent;

    //-- Write as many bytes as possible, return false on errors.
    if (!iCanBus->canWrite(canBuffer, preparedMessages, &sent))
    {
        //-- Something bad happened, try again on the next call.
        return;
    }

    if (dumpWriter)
    {
        for (int i = 0; i < sent; i++)
        {
            dumpMessage(canBuffer[i]);
        }
    }

    //-- Some messages could not be sent, preserve them for later.
    if (sent != preparedMessages)
    {
        handlePartialWrite(sent);
    }

    preparedMessages -= sent;
}

// -----------------------------------------------------------------------------

void CanWriterThread::run()
{
    while (!isStopping())
    {
        //-- Lend CPU time to read threads.
        // https://github.com/roboticslab-uc3m/yarp-devices/issues/191
        yarp::os::Time::delay(delay);

        //-- Send everything and reset the queue.
        flush();
    }
}

// -----------------------------------------------------------------------------

void CanWriterThread::handlePartialWrite(unsigned int sent)
{
    for (int i = sent, j = 0; i < preparedMessages; i++, j++)
    {
        yarp::dev::CanMessage & msg = canBuffer[j];
        const yarp::dev::CanMessage & pendingMsg = canBuffer[i];

        msg.setId(pendingMsg.getId());
        msg.setLen(pendingMsg.getLen());
        std::memcpy(msg.getData(), pendingMsg.getData(), pendingMsg.getLen());
    }
}

// -----------------------------------------------------------------------------

CanSenderDelegate * CanWriterThread::getDelegate()
{
    return sender;
}

// -----------------------------------------------------------------------------
