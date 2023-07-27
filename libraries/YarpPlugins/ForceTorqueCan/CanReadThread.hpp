// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __CAN_READ_THREAD_HPP__
#define __CAN_READ_THREAD_HPP__

#include <mutex>

#include <yarp/os/Thread.h>
#include <yarp/sig/Vector.h>

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
    yarp::sig::Vector getMeasurements() const;

private:
    void interpretMessage(std::uint8_t op, std::int16_t val1, std::int16_t val2, std::int16_t val3);

    yarp::dev::ICanBus * iCanBus {nullptr};
    yarp::dev::ICanBufferFactory * iCanBufferFactory {nullptr};
    yarp::dev::CanBuffer canBuffer;

    mutable std::mutex msgMutex;

    double fx {0.0};
    double fy {0.0};
    double fz {0.0};

    double mx {0.0};
    double my {0.0};
    double mz {0.0};
};

} // namespace roboticslab

#endif // __CAN_READ_THREAD_HPP__
