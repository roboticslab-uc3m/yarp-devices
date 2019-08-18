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

SdoSemaphore::key_t SdoSemaphore::makeIndexPair(const uint8_t * raw)
{
    uint16_t index = raw[1] + ((uint16_t)raw[2] << 8);
    uint8_t subindex = raw[3];
    return std::make_pair(index, subindex);
}

bool SdoSemaphore::await(sdo_data & msg)
{
    size_t len;
    return await(msg, &len);
}

bool SdoSemaphore::await(sdo_data & data, size_t * len)
{
    if (!active)
    {
        return false;
    }

    const key_t & key = makeIndexPair(data);
    value_t value;

    {
        std::lock_guard<std::mutex> lock(registryMutex);
        std::map<key_t, value_t>::iterator it = registry.find(key);

        if (it == registry.end())
        {
            value.sem = new yarp::os::Semaphore(0);
            value.data = data;
            it = registry.insert(std::make_pair(key, value)).first;
        }
        else
        {
            value.sem = it->second.sem;
        }
    }

    bool timedOut = !value.sem->waitWithTimeout(timeout);

    {
        std::lock_guard<std::mutex> lock(registryMutex);

        if (!timedOut)
        {
            *len = value.len;
        }

        if (!value.sem->check()) // nobody is using this semaphore right now
        {
            delete value.sem;
            registry.erase(key);
        }
    }

    return !timedOut;
}

void SdoSemaphore::notify(const uint8_t * raw, size_t len)
{
    if (!active)
    {
        return;
    }

    const key_t key = makeIndexPair(raw);
    std::lock_guard<std::mutex> lock(registryMutex);
    std::map<key_t, value_t>::iterator it = registry.find(key);

    if (it != registry.end())
    {
        std::memcpy(it->second.data.storage, raw, len);
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
