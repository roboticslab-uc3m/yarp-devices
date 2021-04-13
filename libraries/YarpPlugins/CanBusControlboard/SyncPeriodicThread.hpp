// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __SYNC_PERIODIC_THREAD_HPP__
#define __SYNC_PERIODIC_THREAD_HPP__

#include <vector>

#include <yarp/os/PeriodicThread.h>

#include "CanBusBroker.hpp"
#include "FutureTask.hpp"

namespace roboticslab
{

/**
 * @ingroup CanBusControlboard
 * @brief Periodic SYNC signal emitter.
 *
 * This thread performs periodic synchronization tasks across all managed
 * subdevices and sends a SYNC signal at the end of each iteration.
 */
class SyncPeriodicThread final : public yarp::os::PeriodicThread
{
public:
    //! Constructor, manages the lifetime of @ref taskFactory.
    SyncPeriodicThread(std::vector<CanBusBroker *> & canBusBrokers, FutureTaskFactory * taskFactory);

    //! Destructor.
    ~SyncPeriodicThread();

    //! Periodic task.
    void run() override;

private:
    std::vector<CanBusBroker *> & canBusBrokers;
    FutureTaskFactory * taskFactory;
};

} // namespace roboticslab

#endif // __SYNC_PERIODIC_THREAD_HPP__
