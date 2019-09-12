// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "StateObserver.hpp"

#include <cstring>

#include <atomic>
#include <mutex>

#include <yarp/os/Semaphore.h>

using namespace roboticslab;

class StateObserverBase::Private
{
public:
    Private(double timeout)
        : timeout(timeout), semaphore(nullptr), remoteStorage(nullptr), active(true)
    { }

    ~Private()
    {
        interrupt();
    }

    bool await(void * raw)
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

    bool notify(const void * raw, std::size_t len)
    {
        if (!active)
        {
            return false;
        }

        std::lock_guard<std::mutex> lock(registryMutex);

        if (semaphore != nullptr)
        {
            if (raw != nullptr)
            {
                std::memcpy(remoteStorage, raw, len);
            }

            semaphore->post();
            return true;
        }

        return false;
    }

    void interrupt()
    {
        active = false;
        std::lock_guard<std::mutex> lock(registryMutex);

        if (semaphore != nullptr)
        {
            semaphore->post();
        }
    }

private:
    double timeout;
    yarp::os::Semaphore * semaphore;
    void * remoteStorage;

    std::atomic_bool active;
    mutable std::mutex registryMutex;
    mutable std::mutex awaitMutex;
};

StateObserverBase::StateObserverBase(double timeout)
    : impl(new Private(timeout))
{ }

StateObserverBase::~StateObserverBase()
{
    delete impl;
}

bool StateObserverBase::await(void * raw)
{
    return impl->await(raw);
}

bool StateObserverBase::notify(const void * raw, std::size_t len)
{
    return impl->notify(raw, len);
}
