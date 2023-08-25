// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __SYNC_PERIODIC_THREAD_HPP__
#define __SYNC_PERIODIC_THREAD_HPP__

#include <string>
#include <vector>

#include <yarp/os/Bottle.h>
#include <yarp/os/PeriodicThread.h>
#include <yarp/os/Port.h>
#include <yarp/os/PortWriterBuffer.h>

#include "CanBusBroker.hpp"
#include "FutureTask.hpp"
#include "StateObserver.hpp"

namespace roboticslab
{

/**
 * @ingroup CanBusControlBoard
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
    ~SyncPeriodicThread() override;

    //! Open synchronization port.
    bool openPort(const std::string & name);

    //! Set synchronization observer.
    void setObserver(TypedStateObserver<double> * syncObserver)
    { this->syncObserver = syncObserver; }

    //! Periodic task.
    void run() override;

private:
    std::vector<CanBusBroker *> & canBusBrokers;
    FutureTaskFactory * taskFactory;
    TypedStateObserver<double> * syncObserver;
    yarp::os::Port syncPort;
    yarp::os::PortWriterBuffer<yarp::os::Bottle> syncWriter;
};

} // namespace roboticslab

#endif // __SYNC_PERIODIC_THREAD_HPP__
