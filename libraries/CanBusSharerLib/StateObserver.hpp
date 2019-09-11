// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __STATE_OBSERVER_HPP__
#define __STATE_OBSERVER_HPP__

#include <cstdint>
#include <cstdlib>

#include <atomic>
#include <mutex>

#include <yarp/os/Semaphore.h>

namespace roboticslab
{

class StateObserver
{
public:
    StateObserver(double timeout);
    ~StateObserver();

    bool await(std::uint8_t * raw);
    bool notify(const std::uint8_t * raw, std::size_t len);
    void interrupt();

private:
    double timeout;
    yarp::os::Semaphore * semaphore;
    std::uint8_t * remoteStorage;

    std::atomic_bool active;
    mutable std::mutex registryMutex;
    mutable std::mutex awaitMutex;
};

} // namespace roboticslab

#endif // __STATE_OBSERVER_HPP__
