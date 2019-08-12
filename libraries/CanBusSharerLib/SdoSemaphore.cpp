// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "SdoSemaphore.hpp"

#include <cassert>

using namespace roboticslab;

SdoSemaphore::SdoSemaphore(double _timeout)
    : timeout(_timeout),
      active(true)
{
    assert(("timeout > 0.0", timeout > 0.0));
}

SdoSemaphore::~SdoSemaphore()
{
    interrupt();
}

bool SdoSemaphore::await(uint16_t index, uint8_t subindex)
{
    if (!active)
    {
        return false;
    }

    yarp::os::Semaphore * semaphore;
    std::pair<uint16_t, uint8_t> key = std::make_pair(index, subindex);

    {
        std::lock_guard<std::mutex> lock(registryMutex);
        auto it = registry.find(key);

        if (it == registry.end())
        {
            semaphore = new yarp::os::Semaphore(0);
            registry.insert(std::make_pair(key, semaphore));
        }
        else
        {
            semaphore = it->second;
        }
    }

    bool timedOut = semaphore->waitWithTimeout(timeout);

    {
        std::lock_guard<std::mutex> lock(registryMutex);

        if (!semaphore->check()) // nobody is using this semaphore right now
        {
            delete semaphore;
            registry.erase(key);
            return true;
        }
    }

    return timedOut;
}

void SdoSemaphore::notify(uint16_t index, uint8_t subindex)
{
    if (!active)
    {
        return;
    }

    std::pair<uint16_t, uint8_t> key = std::make_pair(index, subindex);
    std::lock_guard<std::mutex> lock(registryMutex);
    auto it = registry.find(key);

    if (it != registry.end())
    {
        it->second->post();
    }
}

void SdoSemaphore::notify(uint8_t * canData)
{
    uint16_t index = canData[1] + ((uint16_t)canData[2] << 8);
    uint8_t subindex = canData[3];
    notify(index, subindex);
}

void SdoSemaphore::interrupt()
{
    active = false;

    do
    {
        std::lock_guard<std::mutex> lock(registryMutex);

        for (auto it : registry)
        {
            it.second->post();
        }
    }
    while (!registry.empty());
}
