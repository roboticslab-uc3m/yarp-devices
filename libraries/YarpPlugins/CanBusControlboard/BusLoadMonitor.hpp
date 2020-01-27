// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __BUS_LOAD_MONITOR_HPP__
#define __BUS_LOAD_MONITOR_HPP__

#include <atomic>

#include <yarp/os/Bottle.h>
#include <yarp/os/PeriodicThread.h>
#include <yarp/os/PortWriterBuffer.h>

#include "CanMessageNotifier.hpp"

namespace roboticslab
{

/**
 * @ingroup CanBusControlboard
 * @brief Periodically sends CAN bus load stats through a YARP port.
 */
class BusLoadMonitor final : public yarp::os::PeriodicThread,
                             public yarp::os::PortWriterBuffer<yarp::os::Bottle>,
                             public CanMessageNotifier
{
public:
    //! Constructor.
    BusLoadMonitor(double period) : yarp::os::PeriodicThread(period), bitrate(1.0)
    { }

    //! Tell observers a new CAN message has arrived.
    virtual bool notifyMessage(const can_message & msg) override;

    void setBitrate(unsigned int bitrate)
    { this->bitrate = bitrate; }

protected:
    //! The thread will invoke this periodically.
    virtual void run() override;

private:
    unsigned int bitrate;
    std::atomic<unsigned long> bits;
};

} // namespace roboticslab

#endif // __BUS_LOAD_MONITOR_HPP__
