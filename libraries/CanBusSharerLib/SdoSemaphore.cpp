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

SdoSemaphore::key_t SdoSemaphore::makeIndexPair(const uint8_t * msg)
{
    uint16_t index = msg[1] + ((uint16_t)msg[2] << 8);
    uint8_t subindex = msg[3];
    return std::make_pair(index, subindex);
}

bool SdoSemaphore::await(uint8_t * msg)
{
    size_t len;
    return await(msg, &len);
}

bool SdoSemaphore::await(uint8_t * data, size_t * len)
{
    if (!active)
    {
        return false;
    }

    const key_t & key = makeIndexPair(data);
    Item item;

    {
        std::lock_guard<std::mutex> lock(registryMutex);
        std::map<key_t, Item>::iterator it = registry.find(key);

        if (it == registry.end())
        {
            item.sem = new yarp::os::Semaphore(0);
            item.data = data;
            it = registry.insert(std::make_pair(key, item)).first;
        }
        else
        {
            item.sem = it->second.sem;
        }
    }

    bool timedOut = !item.sem->waitWithTimeout(timeout);

    {
        std::lock_guard<std::mutex> lock(registryMutex);

        if (!timedOut)
        {
            *len = item.len;
        }

        if (!item.sem->check()) // nobody is using this semaphore right now
        {
            delete item.sem;
            registry.erase(key);
        }
    }

    return !timedOut;
}

void SdoSemaphore::notify(const uint8_t * data, size_t len)
{
    if (!active)
    {
        return;
    }

    const key_t key = makeIndexPair(data);
    std::lock_guard<std::mutex> lock(registryMutex);
    std::map<key_t, Item>::iterator it = registry.find(key);

    if (it != registry.end())
    {
        std::memcpy(it->second.data, data, len);
        it->second.len = len;
        it->second.sem->post();
    }
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
