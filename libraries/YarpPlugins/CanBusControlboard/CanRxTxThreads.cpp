// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "CanBusControlboard.hpp"

#include <limits>

#include <yarp/os/Time.h>

using namespace roboticslab;

CanReaderThread::CanReaderThread(const std::string & id,
        const std::map<int, int> & _idxFromCanId,
        const std::vector<ICanBusSharer *> & _iCanBusSharer)
    : CanReaderWriterThread("read", id),
      idxFromCanId(_idxFromCanId),
      iCanBusSharer(_iCanBusSharer)
{}

void CanReaderThread::run()
{
    CD_INFO("Started CanBusControlboard reading thread run.\n");

    while (!isStopping())
    {
        //-- Lend CPU time to write threads.
        // https://github.com/roboticslab-uc3m/yarp-devices/issues/191
        yarp::os::Time::delay(std::numeric_limits<double>::min());

        unsigned int read;

        //-- Blocks with timeout until a message arrives, returns false on errors.
        bool ok = iCanBus->canRead(canBuffer, bufferSize, &read);

        //-- All debugging messages should be contained in canRead, so just loop again.
        if (!ok || read == 0) continue;

        for (int i = 0; i < read; i++)
        {
            const yarp::dev::CanMessage & msg = canBuffer[i];
            int canId = msg.getId() & 0x7F;

            std::map<int, int>::const_iterator idxFromCanIdFound = idxFromCanId.find(canId);

            if (idxFromCanIdFound == idxFromCanId.end())  //-- Can ID not found
            {
                //-- Intercept 700h 0 msg that just indicates presence.
                if (msg.getId() - canId == 0x700)
                {
                    CD_SUCCESS("Device %d indicating presence. %s\n", canId);
                }

                continue;
            }

            iCanBusSharer[idxFromCanIdFound->second]->interpretMessage(msg);
        }
    }

    CD_INFO("Stopping CanBusControlboard reading thread run.\n");
}

// -----------------------------------------------------------------------------

CanWriterThread::CanWriterThread(const std::string & id)
    : CanReaderWriterThread("write", id),
      sender(0),
      preparedMessages(0)
{}

// -----------------------------------------------------------------------------

CanWriterThread::~CanWriterThread()
{
    if (sender)
    {
        delete sender;
        sender = 0;
    }
}

// -----------------------------------------------------------------------------

void CanWriterThread::run()
{
    CD_INFO("Started CanBusControlboard writing thread run.\n");

    while (!isStopping())
    {
        unsigned int sent;

        bufferMutex.lock();

        //-- Blocks with timeout until a message is sent, returns false on errors.
        iCanBus->canWrite(canBuffer, preparedMessages, &sent);
        preparedMessages = 0;

        bufferMutex.unlock();
    }

    CD_INFO("Stopping CanBusControlboard writing thread run.\n");
}

// -----------------------------------------------------------------------------

void CanWriterThread::setCanHandles(yarp::dev::ICanBus * iCanBus, yarp::dev::ICanBufferFactory * iCanBufferFactory, int bufferSize)
{
    CanReaderWriterThread::setCanHandles(iCanBus, iCanBufferFactory, bufferSize);
    sender = new CanSenderDelegate(canBuffer, bufferMutex, preparedMessages, bufferSize);
}

// -----------------------------------------------------------------------------

CanSenderDelegate * CanWriterThread::getDelegate()
{
    return sender;
}

// -----------------------------------------------------------------------------
