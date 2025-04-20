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
#include "Jr3Mbed_ParamsParser.h"

/**
 * @ingroup YarpPlugins
 * @defgroup Jr3Mbed
 * @brief Contains Jr3Mbed.
 */

 /**
 * @ingroup Jr3Mbed
 * @brief Implementation of a CAN node on an Mbed board that publishes data from a JR3 sensor.
 */
class Jr3Mbed : public yarp::dev::DeviceDriver,
                public yarp::dev::ISixAxisForceTorqueSensors,
                public roboticslab::ICanBusSharer,
                public Jr3Mbed_ParamsParser
{
public:
    //  --------- DeviceDriver Declarations. Implementation in DeviceDriverImpl.cpp ---------

    bool open(yarp::os::Searchable & config) override;
    bool close() override;

    //  --------- ICanBusSharer declarations. Implementation in LacqueyFetch.cpp ---------

    unsigned int getId() override;
    bool notifyMessage(const roboticslab::can_message & message) override;
    bool initialize() override;
    bool finalize() override;
    bool registerSender(roboticslab::ICanSenderDelegate * sender) override;
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
        ACK            = 0x100,
        START_SYNC     = 0x180,
        START_ASYNC    = 0x200,
        STOP           = 0x280,
        ZERO_OFFS      = 0x300,
        SET_FILTER     = 0x380,
        GET_STATE      = 0x400,
        GET_FS_FORCES  = 0x480,
        GET_FS_MOMENTS = 0x500,
        RESET          = 0x580,
        FORCES         = 0x600,
        MOMENTS        = 0x680,
        BOOTUP         = 0x700,
    };

    // keep this in sync with the firmware
    enum class jr3_state : std::uint8_t
    { READY = 0x00, NOT_INITIALIZED = 0x01 };

    enum class jr3_mode
    { SYNC, ASYNC, INVALID };

    constexpr unsigned int getCommandId(can_ops op) const
    { return m_canId + static_cast<unsigned int>(op); }

    bool performRequest(const std::string & cmd, const roboticslab::can_message & msg, std::uint8_t * response, bool quiet = false);
    bool sendStartSyncCommand(double filter);
    bool sendStartAsyncCommand(double filter, double delay);
    bool sendCommand(const std::string & cmd, can_ops op);
    bool ping();
    bool queryFullScales();

    bool monitorWorker(const yarp::os::YarpTimerEvent & event);

    std::array<double, 6> scales;
    bool shouldQueryFullScales {false};

    jr3_mode mode {jr3_mode::INVALID};

    roboticslab::ICanSenderDelegate * sender {nullptr};
    roboticslab::TypedStateObserver<std::uint8_t[]> * ackStateObserver {nullptr};

    mutable std::mutex mtx;

    std::array<std::int16_t, 3> buffer {}; // zero-initialize
    std::array<std::int16_t, 6> raw {}; // zero-initialize

    std::atomic<yarp::dev::MAS_status> status {yarp::dev::MAS_UNKNOWN};
    std::atomic_bool isBooting {false};
    double timestamp {0.0};
    std::uint16_t frameCounter {0};
    double lastDiagnosticsTimestamp {0.0};
    unsigned int lastFrameCounter {0};

    yarp::os::Timer * monitorThread {nullptr};

    static constexpr unsigned int FULL_SCALE = 16384; // 2^14
};

#endif // __JR3_MBED_HPP__
