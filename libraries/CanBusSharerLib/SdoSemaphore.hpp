// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __SDO_SEMAPHORE_HPP__
#define __SDO_SEMAPHORE_HPP__

#include <cstdint>

#include <atomic>
#include <mutex>

#include <yarp/os/Semaphore.h>

namespace roboticslab
{

class SdoSemaphore
{
public:
    SdoSemaphore(double timeout);
    ~SdoSemaphore();

    bool await(std::uint8_t * raw);
    bool notify(const std::uint8_t * raw);
    void interrupt();

private:
    double timeout;
    yarp::os::Semaphore * semaphore;
    std::uint8_t * remoteStorage;

    std::atomic_bool active;
    mutable std::mutex registryMutex;
    mutable std::mutex awaitMutex;
};

}  // namespace roboticslab

#endif // __SDO_SEMAPHORE_HPP__
