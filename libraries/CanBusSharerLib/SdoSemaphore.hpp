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

    bool await(uint8_t * msg); // TODO: remove
    bool await(uint8_t * data, size_t * len);
    void notify(const uint8_t * data, size_t len);
    void interrupt();

private:
    typedef std::pair<uint16_t, uint8_t> key_t;

    struct Item
    { yarp::os::Semaphore * sem; uint8_t * data; size_t len; };

    key_t makeIndexPair(const uint8_t * msg);

    double timeout;
    bool active;
    std::map<key_t, Item> registry;
    mutable std::mutex registryMutex;
};

}  // namespace roboticslab

#endif // __SDO_SEMAPHORE_HPP__
