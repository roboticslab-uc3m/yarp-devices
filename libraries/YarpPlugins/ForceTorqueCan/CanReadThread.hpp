// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __CAN_READ_THREAD_HPP__
#define __CAN_READ_THREAD_HPP__

#include <yarp/os/Thread.h>

#include <yarp/dev/CanBusInterface.h>

namespace roboticslab
{

/**
 * @ingroup ForceTorqueCan
 * @brief A thread for reading incoming CAN messages.
 */
class CanReadThread : public yarp::os::Thread
{
public:
    CanReadThread(yarp::dev::ICanBus * _iCanBus, yarp::dev::ICanBufferFactory * _iCanBufferFactory)
        : iCanBus(_iCanBus), iCanBufferFactory(_iCanBufferFactory)
    { }

    bool threadInit() override;
    void threadRelease() override;
    void run() override;

private:
    yarp::dev::ICanBus * iCanBus {nullptr};
    yarp::dev::ICanBufferFactory * iCanBufferFactory {nullptr};
    yarp::dev::CanBuffer canBuffer;
};

} // namespace roboticslab

#endif // __CAN_READ_THREAD_HPP__
