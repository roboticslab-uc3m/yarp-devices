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

    bool await(uint16_t index, uint8_t subindex = 0x00);
    void notify(uint16_t index, uint8_t subindex);
    void notify(uint8_t * canData);
    void interrupt();

private:
    double timeout;
    bool active;
    std::map<std::pair<uint16_t, uint8_t>, yarp::os::Semaphore *> registry;
    mutable std::mutex registryMutex;
};

}  // namespace roboticslab

#endif // __SDO_SEMAPHORE_HPP__
