// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __CUI_ABSOLUTE_HPP__
#define __CUI_ABSOLUTE_HPP__

#include <cstdint>

#include <mutex>
#include <string>

#include <yarp/conf/numeric.h>

#include <yarp/dev/DeviceDriver.h>
#include <yarp/dev/IEncodersTimed.h>

#include "ICanBusSharer.hpp"
#include "StateObserver.hpp"
#include "CuiAbsolute_ParamsParser.h"

#define CHECK_JOINT(j) do { int ax; if (getAxes(&ax), (j) != ax - 1) return false; } while (0)

/**
 * @ingroup YarpPlugins
 * @defgroup CuiAbsolute
 * @brief Contains CuiAbsolute.
 */

/**
 * @ingroup CuiAbsolute
 * @brief Implementation for the Cui Absolute Encoder custom UC3M circuit as a single
 * CAN bus joint (control board raw interfaces).
 */
class CuiAbsolute : public yarp::dev::DeviceDriver,
                    public yarp::dev::IEncodersTimedRaw,
                    public roboticslab::ICanBusSharer,
                    public CuiAbsolute_ParamsParser
{
public:

    //  --------- DeviceDriver declarations. Implementation in DeviceDriverImpl.cpp ---------

    bool open(yarp::os::Searchable & config) override;
    bool close() override;

    //  --------- ICanBusSharer declarations. Implementation in ICanBusSharerImpl.cpp ---------

    unsigned int getId() override;
    bool notifyMessage(const roboticslab::can_message & message) override;
    bool initialize() override;
    bool finalize() override;
    bool registerSender(roboticslab::ICanSenderDelegate * sender) override;
    bool synchronize(double timestamp) override;

    //  ---------- IEncodersRaw declarations. Implementation in IEncodersRawImpl.cpp ----------

    bool getAxes(int * ax) override;
    bool resetEncoderRaw(int j) override;
    bool resetEncodersRaw() override;
    bool setEncoderRaw(int j, double val) override;
    bool setEncodersRaw(const double * vals) override;
    bool getEncoderRaw(int j, double * v) override;
    bool getEncodersRaw(double * encs) override;
    bool getEncoderSpeedRaw(int j, double * sp) override;
    bool getEncoderSpeedsRaw(double * spds) override;
    bool getEncoderAccelerationRaw(int j, double * spds) override;
    bool getEncoderAccelerationsRaw(double * accs) override;

    //  ---------- IEncodersTimedRaw declarations. Implementation in IEncodersRawImpl.cpp ----------

    bool getEncodersTimedRaw(double * encs, double * time) override;
    bool getEncoderTimedRaw(int j, double * encs, double * time) override;

private:

    using encoder_t = yarp::conf::float32_t;

    enum class CuiMode { PUSH, PULL, OFF };
    enum class CuiCommand : std::uint8_t { PUSH_START = 1, PUSH_STOP = 2, POLL = 3 };

    bool performRequest(const std::string & name, unsigned int len, const std::uint8_t * msgData, encoder_t * resp = nullptr);
    bool startPushMode();
    bool stopPushMode();
    bool pollEncoderRead(encoder_t * enc);
    void normalize(encoder_t * v);

    CuiMode cuiMode {CuiMode::OFF};

    encoder_t encoder {};
    double encoderTimestamp {0.0};

    roboticslab::ICanSenderDelegate * sender {nullptr};
    roboticslab::StateObserver * pushStateObserver {nullptr};
    roboticslab::TypedStateObserver<encoder_t> * pollStateObserver {nullptr};

    mutable std::mutex mutex;
};

#endif // __CUI_ABSOLUTE_HPP__
