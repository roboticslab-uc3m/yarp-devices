// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "SyncPeriodicThread.hpp"

#include <yarp/conf/version.h>

using namespace roboticslab;

// -----------------------------------------------------------------------------

SyncPeriodicThread::SyncPeriodicThread(std::vector<CanBusBroker *> & _canBusBrokers, FutureTaskFactory * _taskFactory)
#if YARP_VERSION_MINOR >= 5
    : yarp::os::PeriodicThread(1.0, yarp::os::ShouldUseSystemClock::Yes, yarp::os::PeriodicThreadClock::Absolute),
#else
    : yarp::os::PeriodicThread(1.0, yarp::os::ShouldUseSystemClock::Yes),
#endif
      canBusBrokers(_canBusBrokers),
      taskFactory(_taskFactory)
{}

// -----------------------------------------------------------------------------

SyncPeriodicThread::~SyncPeriodicThread()
{
    delete taskFactory;
}

// -----------------------------------------------------------------------------

void SyncPeriodicThread::run()
{
    auto task = taskFactory->createTask();

    for (auto * canBusBroker : canBusBrokers)
    {
        task->add([canBusBroker]
            {
                auto * reader = canBusBroker->getReader();
                auto * writer = canBusBroker->getWriter();

                for (auto * handle : reader->getHandles())
                {
                    handle->synchronize();
                }

                writer->getDelegate()->prepareMessage({0x80, 0, nullptr}); // SYNC
                writer->flush();
                return true;
            });
    }

    task->dispatch();
}

// -----------------------------------------------------------------------------
