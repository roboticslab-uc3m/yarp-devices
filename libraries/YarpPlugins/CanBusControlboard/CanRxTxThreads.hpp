// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __CAN_RX_TH_THREADS_HPP__
#define __CAN_RX_TH_THREADS_HPP__

#include <map>
#include <vector>

#include <yarp/os/Thread.h>
#include <yarp/dev/CanBusInterface.h>

#include "ICanBusSharer.h"

namespace roboticslab
{

/**
 * @ingroup CanBusControlboard
 */
class CanReaderThread : public yarp::os::Thread
{
public:
    CanReaderThread(yarp::dev::CanBuffer & canBuffer,
            int bufferSize,
            const std::map<int, int> & idxFromCanId,
            const std::vector<ICanBusSharer *> & iCanBusSharer);

    virtual void run();

    void setCanHandle(yarp::dev::ICanBus * iCanBus)
    { this->iCanBus = iCanBus; }

private:
    yarp::dev::CanBuffer & canBuffer;
    const std::map<int, int> & idxFromCanId;
    const std::vector<ICanBusSharer *> & iCanBusSharer;
    yarp::dev::ICanBus * iCanBus;
    int bufferSize;
};

} // namespace roboticslab

#endif // __CAN_RX_TH_THREADS_HPP__
