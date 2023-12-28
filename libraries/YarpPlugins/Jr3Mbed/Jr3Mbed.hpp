// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __JR3_MBED_HPP__
#define __JR3_MBED_HPP__

#include <cstdint>

#include <array>
#include <atomic>
#include <mutex>
#include <string>

#include <yarp/os/Timer.h>

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
    enum class can_ops : std::uint16_t
    {
        ACK         = 0x100,
        START_SYNC  = 0x180,
        START_ASYNC = 0x200,
        STOP        = 0x280,
        ZERO_OFFS   = 0x300,
        SET_FILTER  = 0x380,
        GET_FS      = 0x400,
        GET_STATE   = 0x480,
        RESET       = 0x500,
        FORCES      = 0x580,
        MOMENTS     = 0x600,
        BOOTUP      = 0x700,
    };

    enum class jr3_mode
    { SYNC, ASYNC, INVALID };

    // keep this in sync with the firmware
    enum class jr3_state : std::uint8_t
    { READY = 0x00, NOT_INITIALIZED = 0x01 };

    constexpr unsigned int getCommandId(can_ops op) const
    { return canId + static_cast<unsigned int>(op); }

    bool performRequest(const std::string & cmd, const can_message & msg, bool quiet = false);
    bool sendStartSyncCommand(double filter);
    bool sendStartAsyncCommand(double filter, double delay);
    bool sendCommand(const std::string & cmd, can_ops op);
    bool ping();

    unsigned int canId {0};

    double filter {0.0}; // cutoff frequency [Hz]
    double asyncPeriod {0.0}; // [s]

    std::string name;
    std::array<double, 6> scales;

    jr3_mode mode {jr3_mode::INVALID};

    ICanSenderDelegate * sender {nullptr};
    TypedStateObserver<std::uint8_t> * ackStateObserver {nullptr};

    mutable std::mutex mtx;

    std::array<std::int16_t, 3> buffer {}; // zero-initialize
    std::array<std::int16_t, 6> raw {}; // zero-initialize

    std::atomic<yarp::dev::MAS_status> status {yarp::dev::MAS_UNKNOWN};
    std::atomic_bool isBooting {false};
    double timestamp {0.0};
    std::uint16_t integrityCounter {0};

    yarp::os::Timer * monitorThread {nullptr};
};

} // namespace roboticslab

#endif // __JR3_MBED_HPP__
