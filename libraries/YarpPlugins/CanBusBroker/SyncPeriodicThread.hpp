// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __SYNC_PERIODIC_THREAD_HPP__
#define __SYNC_PERIODIC_THREAD_HPP__

#include <string>
#include <vector>

#include <yarp/os/Bottle.h>
#include <yarp/os/PeriodicThread.h>
#include <yarp/os/Port.h>
#include <yarp/os/PortWriterBuffer.h>

#include "FutureTask.hpp"
#include "SingleBusBroker.hpp"
#include "StateObserver.hpp"

namespace roboticslab
{

/**
 * @ingroup CanBusBroker
 * @brief Periodic SYNC signal emitter.
 *
 * This thread performs periodic synchronization tasks across all managed
 * subdevices and sends a SYNC signal at the end of each iteration.
 */
class SyncPeriodicThread final : public yarp::os::PeriodicThread
{
public:
    //! Constructor.
    SyncPeriodicThread();

    //! Destructor.
    ~SyncPeriodicThread() override;

    //! Register broker.
    void registerBroker(SingleBusBroker * broker);

    //! Retrieve registered brokers.
    const auto & getBrokers() const
    { return brokers; }

    //! Open synchronization port.
    bool openPort(const std::string & name);

    //! Close synchronization port.
    void closePort();

    //! Set synchronization observer.
    void setObserver(TypedStateObserver<double> * syncObserver);

protected:
    void beforeStart() override;
    bool threadInit() override;
    void run() override;

private:
    std::vector<SingleBusBroker *> brokers;
    FutureTaskFactory * taskFactory {nullptr};
    TypedStateObserver<double> * syncObserver {nullptr};
    yarp::os::Port syncPort;
    yarp::os::PortWriterBuffer<yarp::os::Bottle> syncWriter;
};

} // namespace roboticslab

#endif // __SYNC_PERIODIC_THREAD_HPP__
