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
    Private(StateObserverBase & _owner)
        : owner(_owner), semaphore(nullptr), remoteStorage(nullptr), active(true)
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

        bool timedOut = !semaphore->waitWithTimeout(owner.getTimeout());

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
                owner.setRemoteStorage(raw, len);
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

    void * getRemoteStorage()
    {
        return remoteStorage;
    }

    const void * getRemoteStorage() const
    {
        return remoteStorage;
    }

private:
    StateObserverBase & owner;

    yarp::os::Semaphore * semaphore;
    void * remoteStorage;

    std::atomic_bool active;
    std::mutex registryMutex;
    std::mutex awaitMutex;
};

StateObserverBase::StateObserverBase(double _timeout)
    : timeout(_timeout), impl(new Private(*this))
{ }

StateObserverBase::~StateObserverBase()
{
    delete impl;
}

void * StateObserverBase::getRemoteStorage()
{
    return impl->getRemoteStorage();
}

const void * StateObserverBase::getRemoteStorage() const
{
    return impl->getRemoteStorage();
}

void StateObserverBase::setRemoteStorage(const void * raw, std::size_t len)
{
    std::memcpy(impl->getRemoteStorage(), raw, len);
}

bool StateObserverBase::await(void * raw)
{
    return impl->await(raw);
}

bool StateObserverBase::notify(const void * raw, std::size_t len)
{
    return impl->notify(raw, len);
}
