// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __JR3_MBED_HPP__
#define __JR3_MBED_HPP__

#include <cstdint>

#include <mutex>
#include <string>
#include <vector>

#include <yarp/dev/DeviceDriver.h>
#include <yarp/dev/MultipleAnalogSensorsInterfaces.h>

#include "ICanBusSharer.hpp"
#include "StateObserver.hpp"

namespace roboticslab
{

/**
 * @ingroup YarpPlugins
 * @defgroup Jr3Mbed
 * @brief Contains roboticslab::Jr3Mbed.
 */

 /**
 * @ingroup Jr3Mbed
 * @brief Implementation of a CAN node on an Mbed board that publishes data from a JR3 sensor.
 */
class Jr3Mbed : public yarp::dev::DeviceDriver,
                public yarp::dev::ISixAxisForceTorqueSensors,
                public ICanBusSharer
{
public:
    Jr3Mbed()
    { }

    ~Jr3Mbed() override
    { close(); }

    //  --------- DeviceDriver Declarations. Implementation in DeviceDriverImpl.cpp ---------

    bool open(yarp::os::Searchable & config) override;
    bool close() override;

    //  --------- ICanBusSharer declarations. Implementation in LacqueyFetch.cpp ---------

    unsigned int getId() override;
    bool notifyMessage(const can_message & message) override;
    bool initialize() override;
    bool finalize() override;
    bool registerSender(ICanSenderDelegate * sender) override;
    bool synchronize(double timestamp) override;

    //  --------- ISixAxisForceTorqueSensors Declarations. Implementation in ISixAxisForceTorqueSensorsImpl.cpp ---------

    std::size_t getNrOfSixAxisForceTorqueSensors() const override;
    yarp::dev::MAS_status getSixAxisForceTorqueSensorStatus(std::size_t sens_index) const override;
    bool getSixAxisForceTorqueSensorName(std::size_t sens_index, std::string & name) const override;
    bool getSixAxisForceTorqueSensorFrameName(std::size_t sens_index, std::string & name) const override;
    bool getSixAxisForceTorqueSensorMeasure(std::size_t sens_index, yarp::sig::Vector & out, double & timestamp) const override;

private:
    // keep this in sync with the firmware
    enum can_ops
    {
        JR3_BOOTUP = 2,  // 0x100
        JR3_ACK,         // 0x180
        JR3_START_SYNC,  // 0x200
        JR3_START_ASYNC, // 0x280
        JR3_STOP,        // 0x300
        JR3_ZERO_OFFS,   // 0x380
        JR3_SET_FILTER,  // 0x400
        JR3_GET_FORCES,  // 0x480
        JR3_GET_MOMENTS  // 0x500
    };

    enum jr3_mode
    { SYNC, ASYNC, INVALID };

    bool performRequest(const std::string & cmd, const can_message & msg);
    bool sendStartSyncCommand(double filter);
    bool sendStartAsyncCommand(double filter, double delay);
    bool sendStopCommand();

    unsigned int canId {0};
    double filter {0.0}; // cutoff frequency [Hz]
    double asyncPeriod {0.0}; // [s]
    jr3_mode mode {INVALID};
    ICanSenderDelegate * sender {nullptr};
    StateObserver * ackStateObserver {nullptr};
    mutable std::mutex rxMutex;

    std::vector<std::int16_t> rawForces;
    std::vector<std::int16_t> rawMoments;

    std::vector<double> forceScales;
    std::vector<double> momentScales;
};

} // namespace roboticslab

#endif // __JR3_MBED_HPP__
