// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "SyncPeriodicThread.hpp"

#include <cmath> // std::modf

#include <yarp/os/SystemClock.h>

using namespace roboticslab;

// -----------------------------------------------------------------------------

SyncPeriodicThread::SyncPeriodicThread(std::vector<CanBusBroker *> & _canBusBrokers, FutureTaskFactory * _taskFactory)
    : yarp::os::PeriodicThread(1.0, yarp::os::ShouldUseSystemClock::Yes, yarp::os::PeriodicThreadClock::Absolute),
      canBusBrokers(_canBusBrokers),
      taskFactory(_taskFactory),
      syncObserver(nullptr)
{}

// -----------------------------------------------------------------------------

SyncPeriodicThread::~SyncPeriodicThread()
{
    if (syncPort.isOpen())
    {
        syncPort.interrupt();
        syncPort.close();
    }

    delete taskFactory;
}

// -----------------------------------------------------------------------------

bool SyncPeriodicThread::openPort(const std::string & name)
{
    syncPort.setWriteOnly();
    syncWriter.attach(syncPort);
    return syncPort.open(name);
}

// -----------------------------------------------------------------------------

void SyncPeriodicThread::run()
{
    auto now = yarp::os::SystemClock::nowSystem();
    auto task = taskFactory->createTask();

    for (auto * broker : canBusBrokers)
    {
        task->add([broker, now]
            {
                for (auto * handle : broker->getReader()->getHandles())
                {
                    handle->synchronize(now);
                }

                broker->getWriter()->getDelegate()->prepareMessage({0x80, 0, nullptr}); // SYNC
                broker->getWriter()->flush();
                return true;
            });
    }

    task->dispatch();

    if (syncPort.isOpen())
    {
        double sec;
        double nsec = std::modf(now, &sec) * 1e9;

        syncWriter.prepare() = {
            yarp::os::Value(static_cast<std::int32_t>(sec)),
            yarp::os::Value(static_cast<std::int32_t>(nsec))
        };

        syncWriter.write(true);
    }

    if (syncObserver)
    {
        syncObserver->notify(now);
    }
}

// -----------------------------------------------------------------------------
