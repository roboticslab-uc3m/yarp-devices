// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "SdoSemaphore.hpp"

#include <cassert>
#include <cstring>

#include <bitset>

using namespace roboticslab;

const SdoSemaphore::key_t SdoSemaphore::KEY_SEGMENTED = std::make_pair(0xFFFF, 0xFF); // dummy key

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
    const bool expedited = std::bitset<8>(raw[0]).test(1);

    if (expedited)
    {
        uint16_t index = raw[1] + ((uint16_t)raw[2] << 8);
        uint8_t subindex = raw[3];
        return std::make_pair(index, subindex);
    }
    else
    {
        return KEY_SEGMENTED;
    }
}

bool SdoSemaphore::await(uint8_t * raw)
{
    if (!active)
    {
        return false;
    }

    const key_t & key = makeIndexPair(raw);
    value_t value;

    {
        std::lock_guard<std::mutex> lock(registryMutex);
        std::map<key_t, value_t>::iterator it = registry.find(key);

        if (it == registry.end())
        {
            value.sem = new yarp::os::Semaphore(0);
            value.raw = raw;
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

        if (!value.sem->check()) // nobody is using this semaphore right now
        {
            delete value.sem;
            registry.erase(key);
        }
    }

    return !timedOut;
}

void SdoSemaphore::notify(const uint8_t * raw)
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
        std::memcpy(it->second.raw, raw, 8);
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
