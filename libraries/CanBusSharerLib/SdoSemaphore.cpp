// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "SdoSemaphore.hpp"

#include <cassert>
#include <cstring>

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

bool SdoSemaphore::await(uint8_t * data, uint16_t index, uint8_t subindex)
{
    if (!active)
    {
        return false;
    }

    yarp::os::Semaphore * semaphore;
    std::pair<uint16_t, uint8_t> key = std::make_pair(index, subindex);
    std::map<std::pair<uint16_t, uint8_t>, Item>::iterator it;

    {
        std::lock_guard<std::mutex> lock(registryMutex);
        it = registry.find(key);

        if (it == registry.end())
        {
            Item item;
            item.sem = semaphore = new yarp::os::Semaphore(0);
            it = registry.insert(std::make_pair(key, item)).first;
        }
        else
        {
            semaphore = it->second.sem;
        }
    }

    bool timedOut = semaphore->waitWithTimeout(timeout);

    {
        std::lock_guard<std::mutex> lock(registryMutex);

        if (!timedOut)
        {
            std::memcpy(data, it->second.data, 4);
        }

        if (!semaphore->check()) // nobody is using this semaphore right now
        {
            delete semaphore;
            registry.erase(key);
        }
    }

    return timedOut;
}

bool SdoSemaphore::await(uint8_t * msg)
{
    uint16_t index = msg[1] + ((uint16_t)msg[2] << 8);
    uint8_t subindex = msg[3];
    return await(msg + 4, index, subindex);
}

void SdoSemaphore::notify(const uint8_t * data, uint16_t index, uint8_t subindex)
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
        std::memcpy(it->second.data, data, 4);
        it->second.sem->post();
    }
}

void SdoSemaphore::notify(const uint8_t * msg)
{
    uint16_t index = msg[1] + ((uint16_t)msg[2] << 8);
    uint8_t subindex = msg[3];
    notify(msg + 4, index, subindex);
}

void SdoSemaphore::interrupt()
{
    active = false;

    do
    {
        std::lock_guard<std::mutex> lock(registryMutex);

        for (auto it : registry)
        {
            it.second.sem->post();
        }
    }
    while (!registry.empty());
}
