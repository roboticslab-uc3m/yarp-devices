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

    bool await(uint8_t * data, uint16_t index, uint8_t subindex = 0x00);
    bool await(uint8_t * msg);
    void notify(const uint8_t * data, uint16_t index, uint8_t subindex = 0x00);
    void notify(const uint8_t * msg);
    void interrupt();

private:
    struct Item
    { yarp::os::Semaphore * sem; uint8_t data[4]; };

    double timeout;
    bool active;
    std::map<std::pair<uint16_t, uint8_t>, Item> registry;
    mutable std::mutex registryMutex;
};

}  // namespace roboticslab

#endif // __SDO_SEMAPHORE_HPP__
