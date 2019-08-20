// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __SDO_SEMAPHORE_HPP__
#define __SDO_SEMAPHORE_HPP__

#include <stdint.h>

#include <map>
#include <mutex>
#include <utility>

#include <yarp/os/Semaphore.h>

namespace roboticslab
{

class SdoSemaphore
{
public:
    SdoSemaphore(double timeout);
    ~SdoSemaphore();

    bool await(uint8_t * raw);
    void notify(const uint8_t * raw);
    void interrupt();

private:
    typedef std::pair<uint16_t, uint8_t> key_t;

    typedef struct sdo_item
    { yarp::os::Semaphore * sem; uint8_t * raw; }
    value_t;

    key_t makeIndexPair(const uint8_t * raw);

    double timeout;
    bool active;
    std::map<key_t, value_t> registry;
    mutable std::mutex registryMutex;

    static const key_t KEY_SEGMENTED;
};

}  // namespace roboticslab

#endif // __SDO_SEMAPHORE_HPP__
