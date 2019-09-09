// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "SdoSemaphore.hpp"

#include <cassert>
#include <cstring>

using namespace roboticslab;

SdoSemaphore::SdoSemaphore(double _timeout)
    : timeout(_timeout),
      semaphore(nullptr),
      remoteStorage(nullptr),
      active(true)
{
    assert(("timeout > 0.0", timeout > 0.0));
}

SdoSemaphore::~SdoSemaphore()
{
    interrupt();
}

bool SdoSemaphore::await(std::uint8_t * raw)
{
    std::lock_guard<std::mutex> awaitLock(awaitMutex);

    if (!active)
    {
        return false;
    }

    {
        std::lock_guard<std::mutex> registryLock(registryMutex);
        semaphore = new yarp::os::Semaphore(0);
        remoteStorage = raw;
    }

    bool timedOut = !semaphore->waitWithTimeout(timeout);

    {
        std::lock_guard<std::mutex> registryLock(registryMutex);
        delete semaphore;
        semaphore = nullptr;
    }

    return !timedOut;
}

bool SdoSemaphore::notify(const std::uint8_t * raw)
{
    if (!active)
    {
        return false;
    }

    std::lock_guard<std::mutex> lock(registryMutex);

    if (semaphore != nullptr)
    {
        std::memcpy(remoteStorage, raw, 8);
        semaphore->post();
        return true;
    }

    return false;
}

void SdoSemaphore::interrupt()
{
    active = false;
    std::lock_guard<std::mutex> lock(registryMutex);

    if (semaphore != nullptr)
    {
        semaphore->post();
    }
}
