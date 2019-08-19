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
    struct sdo_data
    {
        uint8_t & operator[](int i) { return storage[i]; }
        const uint8_t & operator[](int i) const { return storage[i]; }
        operator uint8_t *() { return storage; }
        uint8_t storage[8];
    };

    SdoSemaphore(double timeout);
    ~SdoSemaphore();

    bool await(sdo_data & data); // TODO: remove
    bool await(sdo_data & data, size_t * len);
    void notify(const uint8_t * raw, size_t len);
    void interrupt();

private:
    typedef std::pair<uint16_t, uint8_t> key_t;

    typedef struct sdo_item
    { yarp::os::Semaphore * sem; uint8_t * raw; size_t len; }
    value_t;

    key_t makeIndexPair(const uint8_t * raw);

    double timeout;
    bool active;
    std::map<key_t, value_t> registry;
    mutable std::mutex registryMutex;
};

}  // namespace roboticslab

#endif // __SDO_SEMAPHORE_HPP__
