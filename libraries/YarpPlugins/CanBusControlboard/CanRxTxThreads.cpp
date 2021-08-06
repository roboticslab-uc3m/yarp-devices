// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "CanRxTxThreads.hpp"

#include <cstring>
#include <utility> // std::move

#include <yarp/os/LogStream.h>
#include <yarp/os/SystemClock.h>

#include "YarpCanSenderDelegate.hpp"
#include "CanBusBroker.hpp"
#include "LogComponent.hpp"

using namespace roboticslab;

// -----------------------------------------------------------------------------

void CanReaderWriterThread::beforeStart()
{
    yCInfo(CBCB) << "Initializing CanBusControlboard" << type << "thread" << id;
}

// -----------------------------------------------------------------------------

void CanReaderWriterThread::afterStart(bool success)
{
    yCInfo(CBCB) << "Configuring CanBusControlboard" << type << "thread" << id << "->" << (success ? "success" : "failure");
}

// -----------------------------------------------------------------------------

void CanReaderWriterThread::onStop()
{
    yCInfo(CBCB) << "Stopping CanBusControlboard" << type << "thread" << id;
}

// -----------------------------------------------------------------------------

void CanReaderWriterThread::dumpMessage(const can_message & msg, yarp::os::Bottle & b)
{
    b.addInt16(msg.id);

    for (int j = 0; j < msg.len; j++)
    {
        b.addInt8(msg.data[j]);
    }
}

// -----------------------------------------------------------------------------

CanReaderThread::CanReaderThread(const std::string & id, double delay, unsigned int bufferSize)
    : CanReaderWriterThread("read", id, delay, bufferSize),
      canMessageNotifier(nullptr)
{ }

// -----------------------------------------------------------------------------

void CanReaderThread::registerHandle(ICanBusSharer * p)
{
    handles.push_back(p);
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
    yarp::os::Bottle dump;

    while (!isStopping())
    {
        //-- Lend CPU time to write threads.
        // https://github.com/roboticslab-uc3m/yarp-devices/issues/191
        yarp::os::SystemClock::delaySystem(delay);

        //-- Return immediately if there is nothing to be read (non-blocking call), return false on errors.
        ok = iCanBus->canRead(canBuffer, bufferSize, &read);

        //-- All debugging messages should be contained in canRead, so just loop again.
        if (!ok || read == 0) continue;

        if (dumpWriter)
        {
            lastStamp.update();
        }

        for (int i = 0; i < read; i++)
        {
            can_message msg {canBuffer[i].getId(), canBuffer[i].getLen(), canBuffer[i].getData()};
            auto it = canIdToHandle.find(msg.id & 0x7F);

            if (it != canIdToHandle.end())
            {
                it->second->notifyMessage(msg);
            }

            if (dumpWriter)
            {
                dumpMessage(msg, dump.addList());
            }

            if (canMessageNotifier)
            {
                canMessageNotifier->notifyMessage(msg);
            }

            if (busLoadMonitor)
            {
                busLoadMonitor->notifyMessage(msg);
            }
        }

        if (dumpWriter && dump.size() != 0)
        {
            std::lock_guard<std::mutex> lock(*dumpMutex);
            dumpPort->setEnvelope(lastStamp);
            dumpWriter->prepare() = std::move(dump);
            dumpWriter->write(true); // wait until any previous sends are complete
            dump.clear();
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
        //-- Something bad happened, abort queue and start anew.
        preparedMessages = 0;
        return;
    }

    if (dumpWriter || busLoadMonitor)
    {
        yarp::os::Bottle dump;

        lastStamp.update();

        for (int i = 0; i < sent; i++)
        {
            can_message msg {canBuffer[i].getId(), canBuffer[i].getLen(), canBuffer[i].getData()};

            if (dumpWriter)
            {
                dumpMessage(msg, dump.addList());
            }

            if (busLoadMonitor)
            {
                busLoadMonitor->notifyMessage(msg);
            }
        }

        if (dumpWriter && dump.size() != 0)
        {
            std::lock_guard<std::mutex> lock(*dumpMutex);
            dumpPort->setEnvelope(lastStamp);
            dumpWriter->prepare() = std::move(dump);
            dumpWriter->write(true); // wait until any previous sends are complete
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
        yarp::os::SystemClock::delaySystem(delay);

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
