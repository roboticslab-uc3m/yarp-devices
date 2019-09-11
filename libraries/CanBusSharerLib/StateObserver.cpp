// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "StateObserver.hpp"

#include <cassert>
#include <cstring>

using namespace roboticslab;

StateObserver::StateObserver(double _timeout)
    : timeout(_timeout),
      semaphore(nullptr),
      remoteStorage(nullptr),
      active(true)
{
    assert(("timeout > 0.0", timeout > 0.0));
}

StateObserver::~StateObserver()
{
    interrupt();
}

bool StateObserver::await(std::uint8_t * raw)
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

bool StateObserver::notify(const std::uint8_t * raw, std::size_t len)
{
    if (!active)
    {
        return false;
    }

    std::lock_guard<std::mutex> lock(registryMutex);

    if (semaphore != nullptr)
    {
        std::memcpy(remoteStorage, raw, len);
        semaphore->post();
        return true;
    }

    return false;
}

void StateObserver::interrupt()
{
    active = false;
    std::lock_guard<std::mutex> lock(registryMutex);

    if (semaphore != nullptr)
    {
        semaphore->post();
    }
}
